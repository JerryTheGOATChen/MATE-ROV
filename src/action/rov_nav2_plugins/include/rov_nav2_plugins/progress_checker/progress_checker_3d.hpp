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

#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/progress_checker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rov_nav2_plugins/common/pose4d.hpp"

namespace rov_nav2_plugins
{

/**
 * @brief Nav2 progress checker that counts movement in 3D, and turning, as progress.
 *
 * Modeled on Orca4's ProgressChecker3D, which measures distance in 3D so that diving and surfacing
 * count as progress (Nav2's SimpleProgressChecker only looks at x and y). Turning by
 * `required_movement_angle` also counts, as in Nav2's PoseProgressChecker, so turning in place to a
 * goal heading is not mistaken for being stuck.
 *
 * Parameters (under the plugin name):
 *  - `required_movement_radius` (m, default 0.5)
 *  - `required_movement_angle` (rad, default 0.5)
 *  - `movement_time_allowance` (s, default 10.0): time allowed to move or turn that much
 */
class ProgressChecker3D : public nav2_core::ProgressChecker
{
public:
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & plugin_name) override;

  bool check(geometry_msgs::msg::PoseStamped & current_pose) override;

  void reset() override;

private:
  bool moved_enough(const Pose4D & pose) const;

  rclcpp::Clock::SharedPtr clock_;
  double required_movement_radius_{0.5};
  double required_movement_angle_{0.5};
  rclcpp::Duration movement_time_allowance_{rclcpp::Duration::from_seconds(10.0)};

  std::optional<Pose4D> baseline_pose_;
  rclcpp::Time baseline_time_;
};

}  // namespace rov_nav2_plugins
