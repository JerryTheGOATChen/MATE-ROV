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
  depth_sub_ = create_subscription<Image>("camera/depth", 10, [this](const Image & msg) { depth_ = msg; });
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
    planned_goal_.reset();
    RCLCPP_INFO(get_logger(), "New game: back to the start");
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
  std::optional<TrackedTarget> best;
  double best_distance = std::numeric_limits<double>::infinity();
  for (const auto & target : tracker_.targets()) {
    const double d = distance(here, target.position);
    if (d < best_distance) {
      best = target;
      best_distance = d;
    }
  }
  return best;
}

void GhostBrainNode::update()
{
  using haunted_house_msgs::msg::GameState;
  if (start_pose_) {
    publish_pose();
  }
  if (!start_pose_ || !map_ || !state_ || state_->phase != GameState::RUNNING) {
    publish_command(geometry_msgs::msg::Twist{});
    return;
  }

  // TODO(hackathon): with nothing in sight, the ghost could look around, or go exploring
  const auto target = choose_target();
  if (!target) {
    publish_command(geometry_msgs::msg::Twist{});
    publish_markers(target);
    return;
  }

  // Plan again for a new target, and now and then anyway, because people keep walking
  const bool new_goal = !planned_goal_ || distance(*planned_goal_, target->position) > 0.5;
  if (new_goal || now() - last_plan_time_ >= replan_period_) {
    path_ = planner_.plan(localizer_.pose().position, target->position, *map_);
    planned_goal_ = target->position;
    last_plan_time_ = now();
    publish_path();
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
