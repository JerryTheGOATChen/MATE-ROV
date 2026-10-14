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
#include <vector>

#include "rov_nav2_plugins/common/pose4d.hpp"

// CostmapPlanner3D's path geometry, independent of Nav2 and ROS communication.

namespace rov_nav2_plugins
{

/**
 * @brief Turn a horizontal path from `start` to `goal` into a 3D path, adding depth and yaw.
 *
 * `xy_path` is a 2D planner's path, such as one routed around obstacles. Its first and last points
 * are replaced by the start's and goal's exact positions. Depth changes in proportion to the
 * horizontal distance travelled. With `z_before_xy`, or when the path does not move horizontally,
 * depth changes first, in place. Yaw turns from the start's to the goal's along the shortest arc, in
 * proportion to the 3D distance travelled. The path always begins with `start` and ends with exactly
 * `goal`, even when their positions coincide (a turn in place).
 *
 * @throws std::invalid_argument if `xy_path` is empty
 */
std::vector<Pose4D> add_depth(
  const std::vector<Eigen::Vector2d> & xy_path, const Pose4D & start, const Pose4D & goal, bool z_before_xy);

}  // namespace rov_nav2_plugins
