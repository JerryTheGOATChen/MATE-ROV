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

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace rov_nav2_plugins
{

/**
 * @brief Nav2 goal checker that requires the horizontal, vertical and yaw errors all within tolerance.
 *
 * Modeled on Orca4's GoalChecker3D, which checks x, y and z. This also checks yaw, so the ROV
 * arrives facing the requested heading (e.g. to point its camera at a target). Position and yaw
 * must hold at the same time, because an ROV can drift off position while it turns.
 *
 * Parameters (under the plugin name):
 *  - `xy_goal_tolerance` (m, default 0.25), `z_goal_tolerance` (m, default 0.25),
 *    `yaw_goal_tolerance` (rad, default 0.25)
 */
class GoalChecker3D : public nav2_core::GoalChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void reset() override;

  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose,
    const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;

  bool getTolerances(geometry_msgs::msg::Pose & pose_tolerance, geometry_msgs::msg::Twist & vel_tolerance) override;

private:
  double xy_goal_tolerance_{0.25};
  double z_goal_tolerance_{0.25};
  double yaw_goal_tolerance_{0.25};
};

}  // namespace rov_nav2_plugins
