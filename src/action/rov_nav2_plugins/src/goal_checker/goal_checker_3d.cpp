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

#include "rov_nav2_plugins/goal_checker/goal_checker_3d.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "angles/angles.h"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rov_nav2_plugins/common/pose4d.hpp"

namespace rov_nav2_plugins
{

void GoalChecker3D::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("GoalChecker3D: unable to lock the parent node");
  }

  xy_goal_tolerance_ = nav2_util::declare_or_get_parameter(node, plugin_name + ".xy_goal_tolerance", 0.25);
  z_goal_tolerance_ = nav2_util::declare_or_get_parameter(node, plugin_name + ".z_goal_tolerance", 0.25);
  yaw_goal_tolerance_ = nav2_util::declare_or_get_parameter(node, plugin_name + ".yaw_goal_tolerance", 0.25);

  if (xy_goal_tolerance_ <= 0.0 || z_goal_tolerance_ <= 0.0 || yaw_goal_tolerance_ <= 0.0) {
    throw std::invalid_argument(plugin_name + ": goal tolerances must be positive");
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Configured %s: tolerances xy %.3f m, z %.3f m, yaw %.3f rad",
    plugin_name.c_str(),
    xy_goal_tolerance_,
    z_goal_tolerance_,
    yaw_goal_tolerance_);
}

void GoalChecker3D::reset()
{}

bool GoalChecker3D::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose,
  const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & /*velocity*/)
{
  const Pose4D query = to_pose4d(query_pose);
  const Pose4D goal = to_pose4d(goal_pose);
  const Eigen::Vector3d error = goal.position - query.position;

  return error.head<2>().norm() <= xy_goal_tolerance_ && std::abs(error.z()) <= z_goal_tolerance_ &&
         std::abs(angles::shortest_angular_distance(query.yaw, goal.yaw)) <= yaw_goal_tolerance_;
}

bool GoalChecker3D::getTolerances(geometry_msgs::msg::Pose & pose_tolerance, geometry_msgs::msg::Twist & vel_tolerance)
{
  // Nav2's convention for "not checked"
  constexpr double kUnchecked = std::numeric_limits<double>::lowest();

  pose_tolerance.position.x = xy_goal_tolerance_;
  pose_tolerance.position.y = xy_goal_tolerance_;
  pose_tolerance.position.z = z_goal_tolerance_;
  pose_tolerance.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw_goal_tolerance_);

  vel_tolerance.linear.x = kUnchecked;
  vel_tolerance.linear.y = kUnchecked;
  vel_tolerance.linear.z = kUnchecked;
  vel_tolerance.angular.x = kUnchecked;
  vel_tolerance.angular.y = kUnchecked;
  vel_tolerance.angular.z = kUnchecked;

  return true;
}

}  // namespace rov_nav2_plugins

PLUGINLIB_EXPORT_CLASS(rov_nav2_plugins::GoalChecker3D, nav2_core::GoalChecker)
