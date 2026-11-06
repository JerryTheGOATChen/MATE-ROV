// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ghost_brain/ghost_brain_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace ghost_brain
{

namespace
{

double distance(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  return std::hypot(a.x - b.x, a.y - b.y, a.z - b.z);
}

/// Points for each kind of target (the game's rules), and roughly how fast the ghost flies (m/s)
constexpr double kPersonPoints = 10.0;
constexpr double kCandyPoints = 1.0;
constexpr double kGhostSpeed = 1.0;

/// Aim this far (m) past a walking person, along the way they are going, so the ghost passes
/// through the catch radius instead of arriving just behind them
constexpr double kLeadMargin = 0.4;
constexpr double kMaxLeadSeconds = 2.0;

/// A person out of view keeps walking straight (their loops are mostly long straight stretches), so
/// keep following where they would be for up to kMaxUnseenSeconds. Only once that runs out, and the
/// ghost is there without seeing them, does it look around for kSearchSeconds before giving up
constexpr double kMaxUnseenSeconds = 4.0;
constexpr double kSearchSeconds = 3.0;

geometry_msgs::msg::Point predict_position(
  const TrackedTarget & target, const geometry_msgs::msg::Point & here, double age_s)
{
  if (!target.is_person) return target.position;
  const double age = std::min(age_s, kMaxUnseenSeconds);

  // Where they are probably standing now: the last sighting, plus the walking since
  geometry_msgs::msg::Point aim;
  aim.x = target.position.x + target.velocity.x * age;
  aim.y = target.position.y + target.velocity.y * age;
  aim.z = target.position.z + target.velocity.z * age;

  // Where they will be when the ghost gets there. Each pass uses the latest aim for the flying time.
  // Never more than kMaxLeadSeconds ahead: people walk loops and turn, so a long guess is wrong
  for (int i = 0; i < 2; ++i) {
    const double time = std::min(distance(here, aim) / kGhostSpeed, kMaxLeadSeconds);
    aim.x = target.position.x + target.velocity.x * (age + time);
    aim.y = target.position.y + target.velocity.y * (age + time);
    aim.z = target.position.z + target.velocity.z * (age + time);
  }

  // A little further on, along their direction of walking (sideways in the plane only)
  const double speed = std::hypot(target.velocity.x, target.velocity.y);
  if (speed > 0.05) {
    aim.x += target.velocity.x / speed * kLeadMargin;
    aim.y += target.velocity.y / speed * kLeadMargin;
  }
  return aim;
}

double target_score(
  const TrackedTarget & target, const geometry_msgs::msg::Point & here,
  const geometry_msgs::msg::Point & aim)
{
  double points = target.is_person ? kPersonPoints : kCandyPoints;

  return points / (1.0 + distance(here, aim));
}

/// The target the ghost is already going for counts this many times better than it scores, so a
/// slightly better one does not make it switch back and forth and never arrive anywhere
constexpr double kStickiness = 1.6;

/// How close (m) the ghost must get to a waypoint, how long (s) it looks around there, and how far
/// (m) around a waypoint counts as seen once it has looked
constexpr double kArriveRadius = 0.7;
constexpr double kLookSeconds = 4.5;
constexpr double kSeenRadius = 1.2;

/// Closer than this (m) to a target, fly straight at it instead of following the planned route
constexpr double kFinalApproach = 1.2;

/// Give up on a target after this long (s) flying straight at it without catching it
constexpr double kApproachSeconds = 4.0;

/// Aim this far (m) above a candy, but never higher than this (the ceiling is at 3 m, and the
/// ghost's 0.3 m radius has to fit under it)
constexpr double kCandyAbove = 0.3;

/// While people are left but none is known, only candy this close (m) is worth stopping the tour for
constexpr double kNearCandy = 2.0;
constexpr double kHighestAim = 2.65;

/// The ghost is a ball this size (m) as far as bumping into things goes
constexpr double kGhostRadius = 0.32;

/// Stuck if it is asked to fly faster than this (m/s) but moves slower than this, for this long (s)
constexpr double kStuckAskedSpeed = 0.3;
constexpr double kStuckMovingSpeed = 0.15;
constexpr double kStuckSeconds = 2.0;

/// Getting out: turn this fast (rad/s) until about half a turn is done (or this long, s), then fly
/// forward at this speed (m/s) for this long (s)
constexpr double kUnstickTurnRate = 1.2;
constexpr double kUnstickTurnTimeout = 4.0;
constexpr double kUnstickForwardSpeed = 0.5;
constexpr double kUnstickForwardSeconds = 1.0;

double yaw_of(const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

/// Spin on the spot, to sweep the camera round the room. It runs for kLookSeconds at each stop of
/// the tour, a bit longer than the 2 * pi / 1.5 = 4.2 s a full turn takes.
geometry_msgs::msg::Twist look_around()
{
  geometry_msgs::msg::Twist spin;
  spin.angular.z = 1.5;  // rad/s, as fast as the ghost turns
  return spin;
}

}  // namespace

GhostBrainNode::GhostBrainNode(const rclcpp::NodeOptions & options)
: LifecycleNode("ghost_brain", options)
{
  declare_parameter<double>("control_rate_hz", 20.0);
  declare_parameter<double>("replan_period_s", 0.5);
  declare_parameter<std::string>("map_frame", "map");
  declare_parameter<std::string>("base_frame", "base_link");
}

GhostBrainNode::CallbackReturn GhostBrainNode::on_configure(const rclcpp_lifecycle::State &)
{
  control_rate_hz_ = get_parameter("control_rate_hz").as_double();
  replan_period_ = rclcpp::Duration::from_seconds(get_parameter("replan_period_s").as_double());
  map_frame_ = get_parameter("map_frame").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  if (control_rate_hz_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "control_rate_hz must be positive");
    return CallbackReturn::FAILURE;
  }

  using geometry_msgs::msg::PoseStamped;
  using geometry_msgs::msg::TwistStamped;
  using haunted_house_msgs::msg::BoxArray;
  using haunted_house_msgs::msg::GameState;
  using nav_msgs::msg::OccupancyGrid;
  using sensor_msgs::msg::CameraInfo;
  using sensor_msgs::msg::Image;
  using sensor_msgs::msg::Imu;
  using sensor_msgs::msg::LaserScan;
  using vision_msgs::msg::Detection2DArray;

  // The house publishes these once and keeps them for anyone who joins later
  const auto latched = rclcpp::QoS(1).transient_local();
  map_sub_ = create_subscription<OccupancyGrid>("map", latched, [this](const OccupancyGrid & msg) { map_ = msg; });
  obstacles_sub_ = create_subscription<BoxArray>("haunted_house/obstacles", latched, [this](const BoxArray & msg) { obstacles_ = msg; });
  start_sub_ = create_subscription<PoseStamped>("haunted_house/start_pose", latched, [this](const PoseStamped & msg) {
    if (!start_pose_) {
      localizer_.reset(msg.pose);
    }
    start_pose_ = msg;
  });

  state_sub_ = create_subscription<GameState>(
    "haunted_house/state", 10, [this](const GameState & msg) { on_state(msg); });
  velocity_sub_ = create_subscription<TwistStamped>(
    "ghost/velocity", 10, [this](const TwistStamped & msg) {
      localizer_.on_velocity(msg);
      remember_pose(rclcpp::Time(msg.header.stamp));
    });
  imu_sub_ = create_subscription<Imu>("ghost/imu", 10, [this](const Imu & msg) {
    localizer_.on_imu(msg);
    remember_pose(rclcpp::Time(msg.header.stamp));
  });
  sonar_sub_ = create_subscription<LaserScan>("sonar/scan", 10, [this](const LaserScan & msg) {
    if (map_) {
      localizer_.on_sonar(msg, *map_);
    }
  });
  depth_sub_ = create_subscription<Image>("camera/depth", 10, [this](const Image & msg) {
    depth_ = msg;
    if (camera_info_) {
      localizer_.on_depth(msg, *camera_info_);
    }
  });
  camera_info_sub_ =
    create_subscription<CameraInfo>("camera/camera_info", 10, [this](const CameraInfo & msg) { camera_info_ = msg; });
  detections_sub_ = create_subscription<Detection2DArray>(
    "camera/detections", 10, [this](const Detection2DArray & msg) { on_detections(msg); });

  cmd_vel_pub_ = create_publisher<TwistStamped>("cmd_vel", 10);
  pose_pub_ = create_publisher<PoseStamped>("ghost_brain/pose", 10);
  path_pub_ = create_publisher<nav_msgs::msg::Path>("ghost_brain/path", 10);
  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("ghost_brain/markers", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  RCLCPP_INFO(get_logger(), "Configured: %.0f Hz", control_rate_hz_);
  return CallbackReturn::SUCCESS;
}

GhostBrainNode::CallbackReturn GhostBrainNode::on_activate(const rclcpp_lifecycle::State &)
{
  cmd_vel_pub_->on_activate();
  pose_pub_->on_activate();
  path_pub_->on_activate();
  markers_pub_->on_activate();
  last_plan_time_ = now();
  const auto period =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / control_rate_hz_));
  timer_ = create_wall_timer(period, [this]() { update(); });

  RCLCPP_INFO(get_logger(), "Activated: boo!");
  return CallbackReturn::SUCCESS;
}

GhostBrainNode::CallbackReturn GhostBrainNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  publish_command(geometry_msgs::msg::Twist{});
  cmd_vel_pub_->on_deactivate();
  pose_pub_->on_deactivate();
  path_pub_->on_deactivate();
  markers_pub_->on_deactivate();

  RCLCPP_INFO(get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

GhostBrainNode::CallbackReturn GhostBrainNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  release_resources();
  RCLCPP_INFO(get_logger(), "Cleaned up");
  return CallbackReturn::SUCCESS;
}

GhostBrainNode::CallbackReturn GhostBrainNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  release_resources();
  RCLCPP_INFO(get_logger(), "Shut down");
  return CallbackReturn::SUCCESS;
}

void GhostBrainNode::on_state(const haunted_house_msgs::msg::GameState & state)
{
  using haunted_house_msgs::msg::GameState;
  // A new game: the ghost is back at the start, and everything has moved
  const bool new_game = state.phase == GameState::WAITING && (!state_ || state_->phase != GameState::WAITING);
  if (new_game && start_pose_) {
    localizer_.reset(start_pose_->pose);
    tracker_.reset();
    path_.clear();
    pose_history_.clear();
    planned_goal_.reset();
    unreachable_.clear();
    explorer_.reset();
    chosen_aim_.reset();
    waypoint_.reset();
    look_until_.reset();
    unstick_ = Unstick::kNone;
    stuck_since_.reset();
    approach_since_.reset();
    approach_target_.reset();
    search_since_.reset();
    RCLCPP_INFO(get_logger(), "New game: back to the start");
  }
  // Standing at the start waiting for the game: the velocity sensor reads only its bias
  localizer_.set_stationary(state.phase == GameState::WAITING);
  // The score tells us when something was collected: forget the nearest one (the ghost's own position
  // estimate is a bit off, so a collected target is not always inside the collect radius)
  if (state_ && !new_game && state.phase == GameState::RUNNING) {
    const auto here = localizer_.pose().position;
    if (state.candies_eaten > state_->candies_eaten) {
      tracker_.forget_nearest(false, here, 1.2);
    }
    if (state.people_haunted > state_->people_haunted) {
      tracker_.forget_nearest(true, here, 1.5);
    }
  }
  state_ = state;
}

void GhostBrainNode::remember_pose(const rclcpp::Time & stamp)
{
  pose_history_.emplace_back(stamp, localizer_.pose());
  while (pose_history_.size() > 200) {
    pose_history_.pop_front();
  }
}

geometry_msgs::msg::Pose GhostBrainNode::pose_at(const rclcpp::Time & stamp) const
{
  // The remembered pose nearest in time, but with the newest heading and position if none is close
  const std::pair<rclcpp::Time, geometry_msgs::msg::Pose> * best = nullptr;
  double best_gap = 0.15;
  for (const auto & entry : pose_history_) {
    const double gap = std::abs((entry.first - stamp).seconds());
    if (gap < best_gap) {
      best_gap = gap;
      best = &entry;
    }
  }
  return best ? best->second : localizer_.pose();
}

void GhostBrainNode::on_detections(const vision_msgs::msg::Detection2DArray & detections)
{
  // The depth image and camera info come from the same frame as the detections, just before them
  if (!depth_ || !camera_info_ || depth_->header.stamp != detections.header.stamp) {
    return;
  }
  tracker_.on_detections(
    detections, *depth_, *camera_info_, pose_at(rclcpp::Time(detections.header.stamp)),
    obstacles_ ? &*obstacles_ : nullptr);
}

double GhostBrainNode::candy_aim_height(const geometry_msgs::msg::Point & candy) const
{
  // Try a little above it, then level with it, then a little below, and take the first spot with
  // room for the ghost (a candy under a chandelier cannot be reached from above)
  for (const double offset : {kCandyAbove, 0.0, -kCandyAbove}) {
    geometry_msgs::msg::Point spot = candy;
    spot.z = std::min(candy.z + offset, kHighestAim);
    if (has_room(spot)) {
      return spot.z;
    }
  }
  return candy.z;
}

bool GhostBrainNode::has_room(const geometry_msgs::msg::Point & p) const
{
  if (p.z < kGhostRadius || !obstacles_) {
    return p.z >= kGhostRadius;
  }
  for (const auto & box : obstacles_->boxes) {
    const double dx = std::max({box.min.x - p.x, 0.0, p.x - box.max.x});
    const double dy = std::max({box.min.y - p.y, 0.0, p.y - box.max.y});
    const double dz = std::max({box.min.z - p.z, 0.0, p.z - box.max.z});
    if (std::hypot(dx, dy, dz) < kGhostRadius) {
      return false;
    }
  }
  return true;
}

std::optional<TrackedTarget> GhostBrainNode::choose_target() const
{
  const auto here = localizer_.pose().position;
  std::optional<TrackedTarget> best;
  double best_score = -std::numeric_limits<double>::infinity();

  // People are worth ten candies. While it knows of any person, it goes after people only (candy
  // it flies past still gets eaten, since passing within the catch radius is enough). While people
  // are left but it knows of none, it tours the house to find them, taking only candy close by
  const bool knows_a_person = std::any_of(
    tracker_.targets().begin(), tracker_.targets().end(), [](const TrackedTarget & t) { return t.is_person; });
  const bool people_left = state_ && state_->people_left > 0;

  for (const auto & target : tracker_.targets()) {
    if (!target.is_person && knows_a_person) {
      continue;
    }
    if (!target.is_person && people_left && distance(here, target.position) > kNearCandy) {
      continue;
    }
    // Skip anything the planner recently found no route to, or the ghost would freeze on it
    const bool skip = std::any_of(unreachable_.begin(), unreachable_.end(), [&](const Unreachable & u) {
      return u.until > now() && distance(u.position, target.position) < 1.0;
    });
    if (skip) {
      continue;
    }

    // Where to aim (a walking person is not where they were seen), and how good a choice it is
    const double age_s = (now() - target.last_seen).seconds();
    const auto aim = predict_position(target, here, age_s);
    double score = target_score(target, here, aim);
    if (chosen_aim_ && distance(*chosen_aim_, aim) < 2.0) {
      score *= kStickiness;  // the one it is already going for
    }
    if (score > best_score) {
      best_score = score;
      best = target;
      best->position = aim;  // the rest of the node plans to this point
      if (!target.is_person) {
        // Candy sits on something (a table, a shelf, the floor): come at it from a little above,
        // still well inside the catch radius, instead of pushing into the side of the furniture
        best->position.z = candy_aim_height(aim);
      }
    }
  }
  return best;
}

bool GhostBrainNode::fly_to(const geometry_msgs::msg::Point & goal)
{
  // Plan again for a new goal, and now and then anyway, because people keep walking
  const bool new_goal = !planned_goal_ || distance(*planned_goal_, goal) > 0.5;
  if (new_goal || now() - last_plan_time_ >= replan_period_) {
    path_ = planner_.plan(localizer_.pose().position, goal, *map_, *obstacles_);
    planned_goal_ = goal;
    last_plan_time_ = now();
    publish_path();
  }
  if (path_.empty()) {
    const auto here = localizer_.pose().position;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "No route from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)",
      here.x, here.y, here.z, goal.x, goal.y, goal.z);
    // No route usually means the ghost itself is wedged somewhere tight (under a table, low in a
    // corner). Back out the way it came and rise toward mid height, until a route turns up
    geometry_msgs::msg::Twist escape;
    escape.linear.x = -0.4;
    escape.linear.z = std::clamp(1.5 - here.z, -0.4, 0.4);
    publish_command(escape);
    return false;
  }
  publish_command(controller_.compute_velocity(localizer_.pose(), path_));
  return true;
}

void GhostBrainNode::explore()
{
  const auto here = localizer_.pose().position;
  explorer_.build(*map_);

  // Looking around at a stop: spin until the time is up, then cross the stop (and what is near) off
  if (look_until_) {
    if (now() < *look_until_) {
      publish_command(look_around());
      return;
    }
    explorer_.mark_visited(here, kSeenRadius);
    look_until_.reset();
    waypoint_.reset();
  }

  // Pick the next stop: the nearest one (planning a route to every stop to compare them would stall
  // the control loop; a stop with no route is skipped once the ghost tries to fly there)
  if (!waypoint_) {
    waypoint_ = explorer_.next_waypoint([&](const geometry_msgs::msg::Point & point) {
      return distance(here, point);
    });
    planned_goal_.reset();
  }
  if (!waypoint_) {
    // The tour is done. People walk about, so if any are left go round again
    if (state_ && state_->people_left > 0) {
      explorer_.reset();
    }
    publish_command(look_around());
    return;
  }

  if (distance(here, *waypoint_) < kArriveRadius) {
    look_until_ = now() + rclcpp::Duration::from_seconds(kLookSeconds);
    return;
  }
  if (!fly_to(*waypoint_)) {
    explorer_.skip(*waypoint_);  // no route: forget this stop
    waypoint_.reset();
  }
}

bool GhostBrainNode::unstick()
{
  const double yaw = yaw_of(localizer_.pose());
  geometry_msgs::msg::Twist cmd;

  switch (unstick_) {
    case Unstick::kNone: {
      // Trying to fly, but the sensor says it is hardly moving: count how long
      const bool trying = last_command_speed_ > kStuckAskedSpeed;
      if (!trying || localizer_.speed() > kStuckMovingSpeed) {
        stuck_since_.reset();
        return false;
      }
      if (!stuck_since_) {
        stuck_since_ = now();
      }
      if ((now() - *stuck_since_).seconds() < kStuckSeconds) {
        return false;
      }
      RCLCPP_WARN(get_logger(), "Stuck: turning around");
      stuck_since_.reset();
      unstick_ = Unstick::kTurn;
      unstick_start_yaw_ = yaw;
      unstick_until_ = now() + rclcpp::Duration::from_seconds(kUnstickTurnTimeout);
      return true;
    }
    case Unstick::kTurn: {
      // Half a turn (the angle between now and the start, about pi) or the timeout
      const double turned = std::abs(std::remainder(yaw - unstick_start_yaw_, 2.0 * M_PI));
      if (turned > M_PI - 0.3 || now() > unstick_until_) {
        unstick_ = Unstick::kForward;
        unstick_until_ = now() + rclcpp::Duration::from_seconds(kUnstickForwardSeconds);
        return true;
      }
      cmd.angular.z = kUnstickTurnRate;
      break;
    }
    case Unstick::kForward:
      if (now() > unstick_until_) {
        // Out: forget the stop it was headed for and the old route, then carry on
        unstick_ = Unstick::kNone;
        waypoint_.reset();
        look_until_.reset();
        planned_goal_.reset();
        path_.clear();
        return false;
      }
      cmd.linear.x = kUnstickForwardSpeed;
      // Stuck on something above or below (a chandelier, the floor): head for mid height too
      cmd.linear.z = std::clamp(1.5 - localizer_.pose().position.z, -0.4, 0.4);
      break;
  }
  publish_command(cmd);
  return true;
}

void GhostBrainNode::update()
{
  using haunted_house_msgs::msg::GameState;
  if (start_pose_) {
    publish_pose();
  }
  if (!start_pose_ || !map_ || !obstacles_ || !state_ || state_->phase != GameState::RUNNING) {
    publish_command(geometry_msgs::msg::Twist{});
    return;
  }

  if (unstick()) {
    publish_markers(std::nullopt);
    return;
  }

  unreachable_.erase(
    std::remove_if(unreachable_.begin(), unreachable_.end(), [&](const Unreachable & u) { return u.until <= now(); }),
    unreachable_.end());

  explorer_.build(*map_);  // the tour needs its waypoints from the start, even while chasing
  const auto target = choose_target();
  chosen_aim_ = target ? std::optional<geometry_msgs::msg::Point>(target->position) : std::nullopt;
  {
    int people = 0, candy = 0;
    for (const auto & tracked : tracker_.targets()) {
      (tracked.is_person ? people : candy) += 1;
    }
    const char * mode = target ? (target->is_person ? "chasing a person" : "chasing candy")
                               : (look_until_ ? "looking around" : "exploring");
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 4000, "%s | knows %d people, %d candy | tour %s", mode, people, candy,
      explorer_.done() ? "done" : "going");
  }
  if (!target) {
    explore();
    publish_markers(target);
    return;
  }

  // Something to chase: drop any half-done look around and go for it
  look_until_.reset();
  waypoint_.reset();

  // At the spot where an out-of-view person went, and they are not in sight: look around for them
  // for a few seconds (if the camera finds them, they are chased again), then give up on them
  const double unseen_s = (now() - target->last_seen).seconds();
  const auto here = localizer_.pose().position;
  if (target->is_person && unseen_s > kMaxUnseenSeconds && distance(here, target->position) < 1.0) {
    if (!search_since_) {
      search_since_ = now();
    }
    if ((now() - *search_since_).seconds() < kSearchSeconds) {
      publish_command(look_around());
      publish_markers(target);
      return;
    }
    RCLCPP_INFO(
      get_logger(), "Lost the person near (%.1f, %.1f, %.1f): looked around, not there", target->position.x,
      target->position.y, target->position.z);
    tracker_.forget_nearest(true, target->position, 2.0);
    search_since_.reset();
    chosen_aim_.reset();
    publish_command(geometry_msgs::msg::Twist{});
    return;
  }
  search_since_.reset();

  // How long since the ghost first got close to THIS target (switching targets restarts the clock)
  const bool close = distance(here, target->position) < kFinalApproach;
  const bool same_target =
    approach_target_ && distance(*approach_target_, target->position) < (target->is_person ? 2.0 : 0.5);
  if (!same_target) {
    approach_since_.reset();
    approach_target_.reset();
  }
  if (close && !approach_since_) {
    approach_since_ = now();
    approach_target_ = target->position;
  } else if (close && (now() - *approach_since_).seconds() > kApproachSeconds &&
    !(target->is_person && unseen_s > 1.5))
  {
    // Right next to it for a while and still not caught. Not seen lately: it was never there (a
    // copy made by position error), so forget it. Seen: it is wedged in furniture, so skip it a while
    RCLCPP_INFO(
      get_logger(), "Giving up on the %s at (%.1f, %.1f, %.1f): %s", target->is_person ? "person" : "candy",
      target->position.x, target->position.y, target->position.z, unseen_s > 1.5 ? "not there" : "cannot reach");
    if (unseen_s > 1.5) {
      tracker_.forget_nearest(false, target->position, 0.5);
    } else {
      unreachable_.push_back({target->position, now() + rclcpp::Duration::from_seconds(60.0)});
    }
    approach_since_.reset();
    approach_target_.reset();
    chosen_aim_.reset();
    return;
  }
  if (approach_target_ && target->is_person) {
    approach_target_ = target->position;  // a person walks: keep up with them
  }

  // Close in and the camera saw it just now: fly straight at it. The planner keeps a safety margin
  // from furniture, which would stop the ghost short of a candy on a table, and a bonk costs nothing.
  // (One that close but unseen may be behind a wall, so that one still gets a planned route)
  if (close && unseen_s < 1.5) {
    path_ = {here, target->position};
    planned_goal_.reset();
    publish_command(controller_.compute_velocity(localizer_.pose(), path_));
    publish_markers(target);
    return;
  }

  if (!fly_to(target->position)) {
    // No route: give up on this one for a few seconds and go for something else
    unreachable_.push_back({target->position, now() + rclcpp::Duration::from_seconds(5.0)});
  }
  publish_markers(target);
}

void GhostBrainNode::publish_command(const geometry_msgs::msg::Twist & twist)
{
  if (!cmd_vel_pub_ || !cmd_vel_pub_->is_activated()) {
    return;
  }
  last_command_speed_ = std::hypot(twist.linear.x, twist.linear.y);
  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = now();
  msg.header.frame_id = base_frame_;
  msg.twist = twist;
  cmd_vel_pub_->publish(msg);
}

void GhostBrainNode::publish_pose()
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = now();
  msg.header.frame_id = map_frame_;
  msg.pose = localizer_.pose();
  pose_pub_->publish(msg);

  geometry_msgs::msg::TransformStamped transform;
  transform.header = msg.header;
  transform.child_frame_id = base_frame_;
  transform.transform.translation.x = msg.pose.position.x;
  transform.transform.translation.y = msg.pose.position.y;
  transform.transform.translation.z = msg.pose.position.z;
  transform.transform.rotation = msg.pose.orientation;
  tf_broadcaster_->sendTransform(transform);
}

void GhostBrainNode::publish_path()
{
  nav_msgs::msg::Path msg;
  msg.header.stamp = now();
  msg.header.frame_id = map_frame_;
  for (const auto & point : path_) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg.header;
    pose.pose.position = point;
    pose.pose.orientation.w = 1.0;
    msg.poses.push_back(pose);
  }
  path_pub_->publish(msg);
}

void GhostBrainNode::publish_markers(const std::optional<TrackedTarget> & target)
{
  using visualization_msgs::msg::Marker;
  visualization_msgs::msg::MarkerArray msg;
  Marker clear;
  clear.header.stamp = now();
  clear.header.frame_id = map_frame_;
  clear.action = Marker::DELETEALL;
  msg.markers.push_back(clear);

  // Everything Tracker knows about: orange for people, green for candy. Add your own decorations here!
  for (const auto & tracked : tracker_.targets()) {
    Marker dot = clear;
    dot.ns = "tracked";
    dot.id = static_cast<int>(msg.markers.size());
    dot.type = Marker::SPHERE;
    dot.action = Marker::ADD;
    dot.pose.position = tracked.position;
    dot.pose.orientation.w = 1.0;
    dot.scale.x = dot.scale.y = dot.scale.z = 0.15;
    dot.color.r = tracked.is_person ? 1.0f : 0.3f;
    dot.color.g = tracked.is_person ? 0.5f : 1.0f;
    dot.color.b = 0.0f;
    dot.color.a = 0.9f;
    msg.markers.push_back(dot);
  }

  // A ring around whatever the ghost is after
  if (target) {
    Marker ring = clear;
    ring.ns = "target";
    ring.id = static_cast<int>(msg.markers.size());
    ring.type = Marker::CYLINDER;
    ring.action = Marker::ADD;
    ring.pose.position = target->position;
    ring.pose.orientation.w = 1.0;
    ring.scale.x = ring.scale.y = 1.0;
    ring.scale.z = 0.02;
    ring.color.r = 0.4f;
    ring.color.g = 1.0f;
    ring.color.b = 0.4f;
    ring.color.a = 0.6f;
    msg.markers.push_back(ring);
  }
  markers_pub_->publish(msg);
}

void GhostBrainNode::release_resources()
{
  timer_.reset();
  velocity_sub_.reset();
  imu_sub_.reset();
  sonar_sub_.reset();
  depth_sub_.reset();
  camera_info_sub_.reset();
  detections_sub_.reset();
  map_sub_.reset();
  start_sub_.reset();
  state_sub_.reset();
  cmd_vel_pub_.reset();
  pose_pub_.reset();
  path_pub_.reset();
  markers_pub_.reset();
  tf_broadcaster_.reset();
  start_pose_.reset();
  map_.reset();
  state_.reset();
  depth_.reset();
  camera_info_.reset();
  path_.clear();
  planned_goal_.reset();
}

}  // namespace ghost_brain

RCLCPP_COMPONENTS_REGISTER_NODE(ghost_brain::GhostBrainNode)
