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

// Ghost style clauded
std_msgs::msg::ColorRGBA rgba(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}
// same body colour as the house's ghost
const std_msgs::msg::ColorRGBA kBodyColor = rgba(0.95f, 0.95f, 1.0f, 0.85f);
// #76CBAC, for the eyes and the trail
const std_msgs::msg::ColorRGBA kGlowColor = rgba(0.463f, 0.796f, 0.675f, 1.0f);
// the eyes flash these on a catch, then fade back to the glow
const std_msgs::msg::ColorRGBA kHauntColor = rgba(1.0f, 0.2f, 0.2f, 1.0f);
const std_msgs::msg::ColorRGBA kCandyColor = rgba(1.0f, 0.85f, 0.2f, 1.0f);
constexpr double kFlashTime = 0.8;   // s for the eyes to fade back
constexpr double kTrailTime = 3.0;   // s of trail
constexpr double kTrailWidth = 0.55;  // m, as wide as the ghost's sheet
constexpr size_t kTrailPieces = 10;   // the fade goes in this many steps
constexpr double kTrailGap = 0.35;    // m, the trail starts this far behind the ghost's center
constexpr double kPopTime = 1.5;     // s a BOO/YUM floats before it's gone

// a blend of two colours: 0 = all a, 1 = all b
std_msgs::msg::ColorRGBA mix(const std_msgs::msg::ColorRGBA & a, const std_msgs::msg::ColorRGBA & b, double k)
{
  const auto f = static_cast<float>(std::clamp(k, 0.0, 1.0));
  return rgba(a.r + f * (b.r - a.r), a.g + f * (b.g - a.g), a.b + f * (b.b - a.b), a.a + f * (b.a - a.a));
}

}  // namespace

GhostBrainNode::GhostBrainNode(const rclcpp::NodeOptions & options)
: LifecycleNode("ghost_brain", options)
{
  declare_parameter<double>("control_rate_hz", 20.0);
  declare_parameter<double>("replan_period_s", 0.5);
  declare_parameter<std::string>("map_frame", "map");
  declare_parameter<std::string>("base_frame", "base_link");
  // Tuning: the defaults come from each class, config/ghost_brain.yaml overrides them
  declare_parameter<double>("tracker.alpha", tracker_.settings().alpha);
  declare_parameter<double>("tracker.beta", tracker_.settings().beta);
  declare_parameter<double>("controller.lookahead_dist", controller_.settings().lookahead_dist);
  declare_parameter<double>("planner.inflation_radius", planner_.settings().inflation_radius);
  declare_parameter<double>("explorer.fade_time_s", explorer_.settings().fade_time);
  declare_parameter<double>("targeting.hysteresis", hysteresis_);
  declare_parameter<double>("targeting.intercept_max_time_s", intercept_max_time_);
}

GhostBrainNode::CallbackReturn GhostBrainNode::on_configure(const rclcpp_lifecycle::State &)
{
  control_rate_hz_ = get_parameter("control_rate_hz").as_double();
  replan_period_ = rclcpp::Duration::from_seconds(get_parameter("replan_period_s").as_double());
  map_frame_ = get_parameter("map_frame").as_string();
  base_frame_ = get_parameter("base_frame").as_string();

  // Tuning
  TrackerSettings tracker_settings = tracker_.settings();
  tracker_settings.alpha = get_parameter("tracker.alpha").as_double();
  tracker_settings.beta = get_parameter("tracker.beta").as_double();
  tracker_.set_settings(tracker_settings);
  ControllerSettings controller_settings = controller_.settings();
  controller_settings.lookahead_dist = get_parameter("controller.lookahead_dist").as_double();
  controller_.set_settings(controller_settings);
  PlannerSettings planner_settings = planner_.settings();
  planner_settings.inflation_radius = get_parameter("planner.inflation_radius").as_double();
  planner_.set_settings(planner_settings);
  ExplorerSettings explorer_settings = explorer_.settings();
  explorer_settings.fade_time = get_parameter("explorer.fade_time_s").as_double();
  explorer_.set_settings(explorer_settings);
  hysteresis_ = get_parameter("targeting.hysteresis").as_double();
  intercept_max_time_ = get_parameter("targeting.intercept_max_time_s").as_double();
  RCLCPP_INFO(
    get_logger(), "Tuning: alpha %.2f, beta %.2f, lookahead %.2f m, inflation %.2f m, fade %.0f s, "
    "hysteresis %.2f, intercept %.1f s", tracker_settings.alpha, tracker_settings.beta,
    controller_settings.lookahead_dist, planner_settings.inflation_radius, explorer_settings.fade_time,
    hysteresis_, intercept_max_time_);

  if (control_rate_hz_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "control_rate_hz must be positive");
    return CallbackReturn::FAILURE;
  }

  using geometry_msgs::msg::PoseStamped;
  using geometry_msgs::msg::TwistStamped;
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
  // The same house in 3D, for candy on furniture
  obstacles_sub_ = create_subscription<haunted_house_msgs::msg::BoxArray>(
    "haunted_house/obstacles", latched, [this](const haunted_house_msgs::msg::BoxArray & msg) {
      planner_.set_obstacles(msg);
      reach_cache_.clear();
    });
  start_sub_ = create_subscription<PoseStamped>("haunted_house/start_pose", latched, [this](const PoseStamped & msg) {
    if (!start_pose_) {
      localizer_.reset(msg.pose);
    }
    start_pose_ = msg;
  });

  state_sub_ = create_subscription<GameState>(
    "haunted_house/state", 10, [this](const GameState & msg) { on_state(msg); });
  velocity_sub_ = create_subscription<TwistStamped>(
    "ghost/velocity", 10, [this](const TwistStamped & msg) { localizer_.on_velocity(msg); });
  imu_sub_ = create_subscription<Imu>("ghost/imu", 10, [this](const Imu & msg) { localizer_.on_imu(msg); });
  sonar_sub_ = create_subscription<LaserScan>("sonar/scan", 10, [this](const LaserScan & msg) {
    if (map_) {
      localizer_.on_sonar(msg, *map_);
    }
  });
  depth_sub_ = create_subscription<Image>("camera/depth", 10, [this](const Image & msg) {
    depth_ = msg;
    // Height: the floor in the depth image says how high we are, so z doesn't drift
    if (camera_info_) {
      localizer_.on_depth(msg, *camera_info_);
    }
    // Exploring: remember what the camera just looked at
    if (camera_info_ && map_) {
      explorer_.on_depth(msg, *camera_info_, localizer_.pose(), *map_);
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
  explored_pub_ = create_publisher<OccupancyGrid>("ghost_brain/explored", 1);
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
  explored_pub_->on_activate();
  last_plan_time_ = now();
  last_explored_time_ = now();
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
  explored_pub_->on_deactivate();

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
    // The ghost is frozen for the countdown, so its velocity readings are pure bias
    localizer_.start_calibration();
    tracker_.reset();
    path_.clear();
    planned_goal_.reset();
    distances_.clear();
    current_target_.reset();
    blacklist_.clear();
    explorer_.reset();
    trail_.clear();
    pops_.clear();
    last_catch_time_.reset();
    // candy moves a little every game
    reach_cache_.clear();
    RCLCPP_INFO(get_logger(), "New game: back to the start");
  }
  if (state_ && state_->phase == GameState::WAITING && state.phase == GameState::RUNNING) {
    localizer_.finish_calibration();
  }
  // Compare to the last state before replacing it: if a count went up, the ghost just caught that
  if (state_) {
    const auto here = localizer_.pose().position;
    for (uint32_t i = state_->people_haunted; i < state.people_haunted; ++i) {
      tracker_.on_collected(true, here);
      // style: BOO, and the eyes flash red
      pops_.push_back({"BOO", here, now(), true});
      last_catch_time_ = now();
      last_catch_person_ = true;
    }
    for (uint32_t i = state_->candies_eaten; i < state.candies_eaten; ++i) {
      tracker_.on_collected(false, here);
      // style: YUM, and the eyes flash yellow
      pops_.push_back({"YUM", here, now(), false});
      last_catch_time_ = now();
      last_catch_person_ = false;
    }
  }
  state_ = state;
}

void GhostBrainNode::on_detections(const vision_msgs::msg::Detection2DArray & detections)
{
  // The depth image and camera info come from the same frame as the detections, just before them
  if (!depth_ || !camera_info_ || depth_->header.stamp != detections.header.stamp) {
    return;
  }
  tracker_.on_detections(detections, *depth_, *camera_info_, localizer_.pose());
}

std::optional<TrackedTarget> GhostBrainNode::choose_target() const
{
  const auto here = localizer_.pose().position;
  const double now_s = now().seconds();
  std::optional<TrackedTarget> best;
  double best_utility = -std::numeric_limits<double>::infinity();
  std::optional<TrackedTarget> current;

  for (const auto & target : tracker_.targets()) {
    // Blacklist
    bool blacklisted = false;
    for (const auto & [spot, when] : blacklist_) {
      if (std::hypot(target.position.x - spot.x, target.position.y - spot.y) < 0.5 && now_s - when.seconds() < 3.0) {
        blacklisted = true;
      }
    }
    if (blacklisted) {
      continue;
    }

    double score = target.is_person ? 10.0 : 1.0;
    // Confidence: a person we haven't seen in a while has probably walked off
    if (target.is_person) {
      score *= std::max(0.0, 1.0 - (now_s - target.last_seen.seconds()) / 10.0);
    }
    double path_length = distance_at(distances_, *map_, target.position);
    // Candy on furniture: the 2D map can't get there, but flying in over (or under) it might
    if (!std::isfinite(path_length) && !target.is_person) {
      if (const auto way = furniture_reach(target.position)) {
        path_length = distance_at(distances_, *map_, way->approach) + distance(way->approach, way->hover);
      }
    }
    if (!std::isfinite(path_length)) {
      continue;  // can't get there
    }
    double dh = std::abs(target.position.z - here.z);
    TrackedTarget candidate = target;
    candidate.utility = score / (path_length/0.8 + dh/0.6 + 1.0);

    if (candidate.utility > best_utility) {
      best = candidate;
      best_utility = candidate.utility;
    }
    // still the one we were going after
    if (current_target_ && target.is_person == current_target_->is_person &&
      distance(target.position, current_target_->position) < 1.0 &&
      (!current || candidate.utility > current->utility))
    {
      current = candidate;
    }
  }

  // Hysteresis: only switch if the new one is a lot better
  if (current && best && best_utility < hysteresis_ * current->utility) {
    return current;
  }
  return best;
}

geometry_msgs::msg::Point GhostBrainNode::intercept(const TrackedTarget & target) const
{
  if (!target.is_person) {
    return target.position;
  }
  // Interception, basic kinematics.
  const auto here = localizer_.pose().position;
  const double rx = target.position.x - here.x;
  const double ry = target.position.y - here.y;
  const double vx = target.velocity.linear.x;
  const double vy = target.velocity.linear.y;
  const double a = vx * vx + vy * vy - 0.8 * 0.8;
  const double b = 2.0 * (rx * vx + ry * vy);
  const double c = rx * rx + ry * ry;
  double t = 0.0;
  if (std::abs(a) < 1e-6) {
    t = std::abs(b) > 1e-6 ? -c / b : 0.0;
  } else {
    const double disc = b * b - 4.0 * a * c;
    if (disc >= 0.0) {
      // a < 0 since people are slower than us, so this root is the positive one
      t = (-b - std::sqrt(disc)) / (2.0 * a);
    }
  }
  t = std::clamp(t, 0.0, intercept_max_time_);

  geometry_msgs::msg::Point aim = target.position;
  // don't aim into a wall
  // walk out from the person in half-cell steps, stop at the last free spot before a wall
  const double reach = std::hypot(vx, vy) * t;
  const int steps = static_cast<int>(std::ceil(reach / (0.5 * map_->info.resolution)));
  for (int i = 1; i <= steps; ++i) {
    const double s = t * i / steps;
    const double x = target.position.x + vx * s;
    const double y = target.position.y + vy * s;
    if (is_occupied(*map_, x, y)) {
      break;
    }
    aim.x = x;
    aim.y = y;
  }
  return aim;
}

std::optional<Reach> GhostBrainNode::furniture_reach(const geometry_msgs::msg::Point & candy) const
{
  // need the 3D house and a flood first, or there's nothing to work it out from (don't cache that)
  if (!planner_.has_obstacles() || !map_ || distances_.empty()) {
    return std::nullopt;
  }
  // worked it out already? The tracker's guess of the candy moves a little as it averages, so near is the same
  for (const auto & [spot, way] : reach_cache_) {
    if (distance(spot, candy) < 0.15) {
      return way;
    }
  }
  const auto way = planner_.reach(candy, distances_, *map_);
  reach_cache_.emplace_back(candy, way);
  return way;
}

void GhostBrainNode::update()
{
  using haunted_house_msgs::msg::GameState;
  if (start_pose_) {
    publish_pose();
    // style: remember where we've been for the trail
    trail_.emplace_back(now(), localizer_.pose().position);
    while ((now() - trail_.front().first).seconds() > kTrailTime) {
      trail_.pop_front();
    }
  }
  // the explored grid for Foxglove, once a second is plenty
  if (now() - last_explored_time_ >= rclcpp::Duration::from_seconds(1.0)) {
    publish_explored();
  }
  if (!start_pose_ || !map_ || !state_ || state_->phase != GameState::RUNNING) {
    publish_command(geometry_msgs::msg::Twist{});
    // keep drawing the ghost in the countdown and after the final whistle
    if (start_pose_) {
      publish_markers(std::nullopt);
    }
    return;
  }

  // Dijkstra flood every replan, so choose_target knows real distances
  if (distances_.empty() || now() - last_flood_time_ >= replan_period_) {
    distances_ = planner_.distances_from(localizer_.pose().position, *map_);
    last_flood_time_ = now();
  }
  // forget blacklist entries after 3 s
  blacklist_.erase(
    std::remove_if(blacklist_.begin(), blacklist_.end(),
      [&](const auto & entry) { return (now() - entry.second).seconds() > 3.0; }),
    blacklist_.end());

  // TODO(hackathon): with nothing in sight, the ghost could look around, or go exploring
  const auto target = choose_target();
  current_target_ = target;
  // Exploring: nothing to chase, so go look somewhere we haven't lately
  std::optional<geometry_msgs::msg::Point> aim =
    target ? std::optional<geometry_msgs::msg::Point>(intercept(*target)) :
    explorer_.next_goal(distances_, localizer_.pose(), now());
  if (!aim) {
    publish_command(geometry_msgs::msg::Twist{});
    publish_markers(target);
    return;
  }

  //Clamp Z above 0.45m above so then it doesn't bonk
  aim->z = std::clamp(aim->z, 0.45, 2.55);

  // Candy on furniture: aim beside the furniture at hover height instead, then slide in at the end
  std::optional<Reach> way;
  if (target && !target->is_person && !std::isfinite(distance_at(distances_, *map_, target->position))) {
    way = furniture_reach(target->position);
    if (way) {
      *aim = way->approach;
    }
  }

  // Plan again for a new target, and now and then anyway, because people keep walking
  const bool new_goal = !planned_goal_ || distance(*planned_goal_, *aim) > 0.5;
  // Final approach: close to a person, replan every tick so we don't overshoot
  const bool close_person = target && target->is_person && distance(localizer_.pose().position, *aim) < 2.0;
  if (new_goal || close_person || now() - last_plan_time_ >= replan_period_) {
    if (way && planner_.line_clear(localizer_.pose().position, way->hover)) {
      // nothing in the way in 3D: fly straight to the hover spot (this is the slide in, too)
      path_ = {localizer_.pose().position, way->hover};
    } else {
      path_ = planner_.plan(localizer_.pose().position, *aim, *map_);
      if (way && !path_.empty()) {
        path_.push_back(way->hover);
      }
    }
    planned_goal_ = *aim;
    last_plan_time_ = now();
    publish_path();
    // couldn't plan there -> skip it for a bit
    if (path_.empty()) {
      RCLCPP_WARN(get_logger(), "No path to (%.2f, %.2f), skipping it", aim->x, aim->y);
      if (target) {
        blacklist_.emplace_back(target->position, now());
      }
      current_target_.reset();
      planned_goal_.reset();
    }
  }

  publish_command(controller_.compute_velocity(localizer_.pose(), path_));
  publish_markers(target);
}

void GhostBrainNode::publish_command(const geometry_msgs::msg::Twist & twist)
{
  if (!cmd_vel_pub_ || !cmd_vel_pub_->is_activated()) {
    return;
  }
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

void GhostBrainNode::publish_explored()
{
  last_explored_time_ = now();
  auto grid = explorer_.seen_grid(now());
  // nothing to show until the first depth frame
  if (grid.data.empty() || !explored_pub_->is_activated()) {
    return;
  }
  grid.header.stamp = now();
  grid.header.frame_id = map_frame_;
  explored_pub_->publish(grid);
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
  draw_ghost(msg);
  markers_pub_->publish(msg);
}

void GhostBrainNode::draw_ghost(visualization_msgs::msg::MarkerArray & msg)
{
  using visualization_msgs::msg::Marker;
  const rclcpp::Time stamp = now();

  // Body and eyes ride along in base_link, so they follow the ghost (same shapes as the house's ghost)
  Marker part;
  part.header.stamp = stamp;
  part.header.frame_id = base_frame_;
  part.frame_locked = true;
  part.ns = "ghost";
  part.action = Marker::ADD;
  part.pose.orientation.w = 1.0;
  const auto add_part = [&](int type, double x, double y, double z, double sx, double sy, double sz,
    const std_msgs::msg::ColorRGBA & color) {
      Marker m = part;
      m.id = static_cast<int>(msg.markers.size());
      m.type = type;
      m.pose.position.x = x;
      m.pose.position.y = y;
      m.pose.position.z = z;
      m.scale.x = sx;
      m.scale.y = sy;
      m.scale.z = sz;
      m.color = color;
      msg.markers.push_back(m);
    };
  add_part(Marker::SPHERE, 0.0, 0.0, 0.1, 0.5, 0.5, 0.5, kBodyColor);        // head
  add_part(Marker::CYLINDER, 0.0, 0.0, -0.15, 0.55, 0.55, 0.4, kBodyColor);  // sheet

  // Eyes: #76CBAC, flashing red (haunt) or yellow (candy) on a catch and fading back
  std_msgs::msg::ColorRGBA eye = kGlowColor;
  if (last_catch_time_) {
    const double since = (stamp - *last_catch_time_).seconds();
    eye = mix(last_catch_person_ ? kHauntColor : kCandyColor, kGlowColor, since / kFlashTime);
  }
  // the glow: a see-through halo around each eye that slowly breathes
  std_msgs::msg::ColorRGBA halo = eye;
  halo.a = 0.3f;
  const double breathe = 0.17 + 0.03 * std::sin(2.0 * M_PI * 0.7 * stamp.seconds());
  for (const double side : {0.09, -0.09}) {
    add_part(Marker::SPHERE, 0.22, side, 0.17, 0.09, 0.09, 0.13, eye);
    add_part(Marker::SPHERE, 0.22, side, 0.17, breathe, breathe, breathe * 1.3, halo);
  }

  // Trail: where it's been, in the map, fading out toward the tail. Drawn as a few pieces with one
  // colour each, since per-point colours on a line came out black in Foxglove
  // start it behind the ghost, not on top of it: skip the newest points that are still under the ghost
  size_t n = trail_.size();
  while (n >= 2 && distance(trail_[n - 1].second, trail_.back().second) < kTrailGap) {
    --n;
  }
  if (n >= 2) {
    const size_t pieces = std::min(kTrailPieces, n - 1);
    for (size_t piece = 0; piece < pieces; ++piece) {
      Marker trail;
      trail.header.stamp = stamp;
      trail.header.frame_id = map_frame_;
      trail.ns = "trail";
      trail.id = static_cast<int>(msg.markers.size());
      trail.type = Marker::LINE_STRIP;
      trail.action = Marker::ADD;
      trail.pose.orientation.w = 1.0;
      trail.scale.x = kTrailWidth;
      trail.color = kGlowColor;
      trail.color.a = static_cast<float>(0.8 * static_cast<double>(piece + 1) / static_cast<double>(pieces));
      // neighbouring pieces share an end point, so there are no gaps
      for (size_t i = piece * (n - 1) / pieces; i <= (piece + 1) * (n - 1) / pieces; ++i) {
        trail.points.push_back(trail_[i].second);
      }
      msg.markers.push_back(trail);
    }
  }

  // BOO / YUM: float up from where the catch happened and fade out
  pops_.erase(
    std::remove_if(pops_.begin(), pops_.end(),
      [&](const Pop & pop) { return (stamp - pop.when).seconds() > kPopTime; }),
    pops_.end());
  for (const auto & pop : pops_) {
    const double age = (stamp - pop.when).seconds();
    Marker text;
    text.header.stamp = stamp;
    text.header.frame_id = map_frame_;
    text.ns = "pops";
    text.id = static_cast<int>(msg.markers.size());
    text.type = Marker::TEXT_VIEW_FACING;
    text.action = Marker::ADD;
    text.pose.position = pop.at;
    text.pose.position.z += 0.6 + 0.4 * age;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.4;
    text.color = pop.is_person ? kHauntColor : kCandyColor;
    text.color.a = static_cast<float>(1.0 - age / kPopTime);
    text.text = pop.text;
    msg.markers.push_back(text);
  }
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
  obstacles_sub_.reset();
  start_sub_.reset();
  state_sub_.reset();
  cmd_vel_pub_.reset();
  pose_pub_.reset();
  path_pub_.reset();
  markers_pub_.reset();
  explored_pub_.reset();
  tf_broadcaster_.reset();
  start_pose_.reset();
  map_.reset();
  state_.reset();
  depth_.reset();
  camera_info_.reset();
  path_.clear();
  planned_goal_.reset();
  distances_.clear();
  current_target_.reset();
  blacklist_.clear();
}

}  // namespace ghost_brain

RCLCPP_COMPONENTS_REGISTER_NODE(ghost_brain::GhostBrainNode)
