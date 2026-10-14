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

#include "rov_nav2_plugins/progress_checker/progress_checker_3d.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include "angles/angles.h"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace rov_nav2_plugins
{

void ProgressChecker3D::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & plugin_name)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("ProgressChecker3D: unable to lock the parent node");
  }

  clock_ = node->get_clock();
  required_movement_radius_ = nav2_util::declare_or_get_parameter(node, plugin_name + ".required_movement_radius", 0.5);
  required_movement_angle_ = nav2_util::declare_or_get_parameter(node, plugin_name + ".required_movement_angle", 0.5);
  const double time_allowance =
    nav2_util::declare_or_get_parameter(node, plugin_name + ".movement_time_allowance", 10.0);

  if (required_movement_radius_ <= 0.0 || required_movement_angle_ <= 0.0 || time_allowance <= 0.0) {
    throw std::invalid_argument(plugin_name + ": movement radius, angle and time allowance must be positive");
  }
  movement_time_allowance_ = rclcpp::Duration::from_seconds(time_allowance);

  RCLCPP_INFO(
    node->get_logger(),
    "Configured %s: move %.2f m or turn %.2f rad within %.1f s",
    plugin_name.c_str(),
    required_movement_radius_,
    required_movement_angle_,
    time_allowance);
}

bool ProgressChecker3D::check(geometry_msgs::msg::PoseStamped & current_pose)
{
  const Pose4D pose = to_pose4d(current_pose.pose);
  const rclcpp::Time now = clock_->now();

  if (!baseline_pose_ || moved_enough(pose)) {
    baseline_pose_ = pose;
    baseline_time_ = now;
    return true;
  }
  return now - baseline_time_ <= movement_time_allowance_;
}

void ProgressChecker3D::reset()
{
  baseline_pose_.reset();
}

bool ProgressChecker3D::moved_enough(const Pose4D & pose) const
{
  return (pose.position - baseline_pose_->position).norm() > required_movement_radius_ ||
         std::abs(angles::shortest_angular_distance(baseline_pose_->yaw, pose.yaw)) > required_movement_angle_;
}

}  // namespace rov_nav2_plugins

PLUGINLIB_EXPORT_CLASS(rov_nav2_plugins::ProgressChecker3D, nav2_core::ProgressChecker)
