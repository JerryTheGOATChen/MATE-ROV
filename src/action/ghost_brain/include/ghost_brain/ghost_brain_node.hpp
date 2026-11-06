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

#pragma once

#include <deque>
#include <memory>
#include <optional>
#include <vector>
#include <string>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "ghost_brain/controller.hpp"
#include "ghost_brain/explorer.hpp"
#include "ghost_brain/localizer.hpp"
#include "ghost_brain/planner.hpp"
#include "ghost_brain/tracker.hpp"
#include "haunted_house_msgs/msg/box_array.hpp"
#include "haunted_house_msgs/msg/game_state.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace ghost_brain
{

/**
 * @brief The ghost's brain: works out where it is and where everything is, then goes after it.
 *
 * Sensor messages go to Localizer (where am I?) and Tracker (where are the people and candy?). While
 * active, a control loop at `control_rate_hz` picks a target from Tracker, plans a route to it with
 * Planner (again every `replan_period_s`, because people move), and sends Controller's velocity on
 * `cmd_vel`. Between games, it sends zero, and at the start of each game it resets Localizer and
 * Tracker.
 *
 * It publishes Localizer's pose as the map -> base_link transform, so Foxglove draws the sonar and
 * camera where the brain thinks the ghost is: when they line up with the house, localization works.
 *
 * Subscribes: `ghost/velocity`, `ghost/imu`, `sonar/scan`, `camera/depth`, `camera/camera_info`,
 * `camera/detections`, `map`, `haunted_house/start_pose`, `haunted_house/state`
 * Publishes: `cmd_vel`, `ghost_brain/pose`, `ghost_brain/path`, `ghost_brain/markers`, and TF
 */
class GhostBrainNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit GhostBrainNode(const rclcpp::NodeOptions & options);

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  /// What to go after next. Starts as the nearest target Tracker knows about: you can do better
  std::optional<TrackedTarget> choose_target() const;

  /// How high to aim for a candy: just above it if there is room for the ghost there, else level
  /// with it or just below
  double candy_aim_height(const geometry_msgs::msg::Point & candy) const;
  bool has_room(const geometry_msgs::msg::Point & p) const;  // no wall, furniture, floor in the way

  void on_state(const haunted_house_msgs::msg::GameState & state);
  void on_detections(const vision_msgs::msg::Detection2DArray & detections);
  void update();

  /// Plan to a point (again now and then) and fly the route. Returns false if there is no route
  bool fly_to(const geometry_msgs::msg::Point & goal);

  /// With nothing worth chasing: tour the house, looking around at each stop
  void explore();

  /// Notice a ghost that is trying to fly but is not moving (pressed against a wall), and get it
  /// out: turn around, fly a little, look around. Returns true while it has taken over
  bool unstick();
  void publish_command(const geometry_msgs::msg::Twist & twist);
  void publish_pose();
  void publish_path();
  void publish_markers(const std::optional<TrackedTarget> & target);
  void release_resources();

  // Parameters
  double control_rate_hz_{20.0};
  rclcpp::Duration replan_period_{rclcpp::Duration::from_seconds(0.5)};
  std::string map_frame_;
  std::string base_frame_;

  // What the ghost knows
  std::optional<geometry_msgs::msg::PoseStamped> start_pose_;
  std::optional<nav_msgs::msg::OccupancyGrid> map_;
  std::optional<haunted_house_msgs::msg::BoxArray> obstacles_;
  std::optional<haunted_house_msgs::msg::GameState> state_;
  std::optional<sensor_msgs::msg::Image> depth_;
  std::optional<sensor_msgs::msg::CameraInfo> camera_info_;

  // What it is doing
  Localizer localizer_;
  Tracker tracker_;
  Planner planner_;
  Explorer explorer_;
  Controller controller_;
  Path path_;
  std::optional<geometry_msgs::msg::Point> planned_goal_;
  std::optional<geometry_msgs::msg::Point> chosen_aim_;  // what it went for last, so it can stick with it
  rclcpp::Time last_plan_time_;

  /// Recent poses and when they were true, so a camera image can use the pose it was taken at
  std::deque<std::pair<rclcpp::Time, geometry_msgs::msg::Pose>> pose_history_;
  void remember_pose(const rclcpp::Time & stamp);
  geometry_msgs::msg::Pose pose_at(const rclcpp::Time & stamp) const;

  /// Targets the planner found no route to (inside a wall, say), and when to try them again
  struct Unreachable
  {
    geometry_msgs::msg::Point position;
    rclcpp::Time until;
  };
  std::vector<Unreachable> unreachable_;
  std::optional<rclcpp::Time> approach_since_;  // when it first got close to the target it is after
  std::optional<geometry_msgs::msg::Point> approach_target_;  // ...and which target that is
  std::optional<rclcpp::Time> search_since_;    // when it started looking for a person who went out of view

  // The tour: where the ghost is headed, and until when it looks around once it gets there
  std::optional<geometry_msgs::msg::Point> waypoint_;
  std::optional<rclcpp::Time> look_until_;

  // Getting unstuck: stuck_since_ is when it started trying to fly without moving
  enum class Unstick { kNone, kTurn, kForward };
  Unstick unstick_{Unstick::kNone};
  double unstick_start_yaw_{0.0};
  rclcpp::Time unstick_until_;
  std::optional<rclcpp::Time> stuck_since_;
  double last_command_speed_{0.0};  // the forward speed last asked for

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sonar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<haunted_house_msgs::msg::BoxArray>::SharedPtr obstacles_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr start_sub_;
  rclcpp::Subscription<haunted_house_msgs::msg::GameState>::SharedPtr state_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace ghost_brain
