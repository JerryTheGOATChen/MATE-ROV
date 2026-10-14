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

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rov_nav2_plugins/controller/controller_math.hpp"
#include "tf2_ros/buffer.hpp"

namespace rov_nav2_plugins
{

/**
 * @brief Nav2 controller: 3D pure pursuit for a holonomic ROV (x, y, z, yaw).
 *
 * Modeled on Orca4's PurePursuitController3D, generalized from diff-drive to holonomic motion and
 * extended to track the path's yaw. See Pursuit for the control law. The costmap is ignored.
 *
 * Parameters (under the plugin name):
 *  - `lookahead_dist` (m), `approach_gain` (1/s), `transform_tolerance` (s)
 *  - `max_vel_x`, `max_vel_y`, `max_vel_z` (m/s), `max_vel_yaw` (rad/s): vehicle-frame limits
 *  - `max_accel_x`, `max_accel_y`, `max_accel_z` (m/s^2), `max_accel_yaw` (rad/s^2)
 */
class PurePursuitController3D : public nav2_core::Controller
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  /// @throws nav2_core::InvalidPath if `path` is empty
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @throws nav2_core::InvalidPath if no path has been set
   * @throws nav2_core::ControllerTFError if `pose` cannot be transformed into the path's frame
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  /// Not supported: Nav2 sends speed limits from costmap filters, which the ROV does not use.
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  void reset() override;

private:
  rclcpp::Logger logger_{rclcpp::get_logger("PurePursuitController3D")};
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string base_frame_;
  std::string path_frame_;
  double transform_tolerance_{0.1};
  double control_period_{0.05};
  std::unique_ptr<Pursuit> pursuit_;
};

}  // namespace rov_nav2_plugins
