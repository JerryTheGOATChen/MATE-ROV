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

#include <Eigen/Core>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rov_kinematic_sim/rov_kinematics.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

namespace rov_kinematic_sim
{

/**
 * @brief Simulated ROV that stands in for the vehicle and its odometry.
 *
 * While active, integrates VehicleModel at a fixed rate under the latest `cmd_vel`, and publishes
 * ground-truth odometry on `odom` and as the odom -> base_link transform. Commands older than
 * `cmd_vel_timeout_s` are dropped, so the vehicle stops if its commander does.
 *
 * Parameters:
 *  - `update_rate_hz` (double): integration and publishing rate
 *  - `cmd_vel_timeout_s` (double): age after which a command is ignored
 *  - `odom_frame`, `base_frame` (string): frames of the published odometry and transform
 *  - `initial_pose` (double[4]): x, y, z (m) and yaw (rad) at configure
 *  - `response_time_s`, `max_linear_speed` (double[3]), `max_yaw_rate`, `water_current` (double[3]),
 *    `surface_z`: the VehicleModel
 */
class KinematicSimNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit KinematicSimNode(const rclcpp::NodeOptions & options);

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  /// @throws std::invalid_argument if a parameter is invalid
  void load_parameters();

  void on_cmd_vel(const geometry_msgs::msg::TwistStamped & msg);
  void update();
  void publish_state(const rclcpp::Time & stamp, const Eigen::Vector3d & body_velocity);
  void release_resources();

  // Parameters
  double update_rate_hz_{50.0};
  rclcpp::Duration cmd_vel_timeout_{rclcpp::Duration::from_seconds(0.5)};
  std::string odom_frame_;
  std::string base_frame_;
  VehicleState initial_state_;
  VehicleModel model_;

  // Simulation state
  VehicleState state_;
  BodyVelocity command_;
  rclcpp::Time command_time_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace rov_kinematic_sim
