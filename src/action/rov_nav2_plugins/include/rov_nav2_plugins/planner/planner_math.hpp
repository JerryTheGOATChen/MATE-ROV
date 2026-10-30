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

#include <cstddef>
#include <cstdint>
#include <Eigen/Core>
#include <functional>
#include <vector>

#include "rov_nav2_plugins/common/pose4d.hpp"
#include "rov_nav2_plugins/costmap/costmap_math.hpp"

// VoxelPlanner3D's path geometry, independent of Nav2 and ROS communication.

namespace rov_nav2_plugins
{

// --- Search ---

struct PlannerConfig
{
  double cost_weight{2.0};  // a step into a voxel of cost c costs (1 + cost_weight * c / 252) times its length
  std::uint8_t unknown_cost{40};  // the least cost of a voxel the sensor has not seen, from 0 to 252
  double z_weight{1.5};  // a meter of depth change counts as this many meters of travel
  double max_planning_time{1.0};  // s
};

enum class PlanStatus
{
  kSucceeded,
  kStartOutside,  // the start is outside the grid
  kGoalOutside,  // the goal is outside the grid
  kStartOccupied,  // the start is in an obstacle
  kGoalOccupied,  // the goal is within the vehicle's size of an obstacle
  kNoPath,  // obstacles block every route
  kTimedOut,
  kCanceled,
};

struct Plan3D
{
  PlanStatus status{PlanStatus::kNoPath};
  std::vector<Eigen::Vector3d> points;  // from exactly the start to exactly the goal; empty unless succeeded
  double search_cost{0.0};  // of the route through voxel centers, before it was shortened
  std::size_t expansions{0};  // voxels the search expanded
};

/**
 * @brief Plan a route through `volume` from `start` to `goal`, over, under or around obstacles.
 *
 * A* over the voxel grid, stepping to any of a voxel's 26 neighbors. A step's length weights depth change by
 * `z_weight`, and the step costs its length times `1 + cost_weight * c / 252`, where c is the cost of the voxel
 * it enters, raised to `unknown_cost` if the sensor has not seen that voxel. The heuristic is the weighted
 * straight-line distance, so the route is the cheapest through the grid.
 *
 * Voxels at the inscribed cost or above, where the vehicle would touch an obstacle, cannot be entered, except
 * that a start in inscribed water may leave through inscribed water, so a vehicle that has drifted close to
 * the floor can still get away from it. A diagonal step may not cut past the edge or corner of a voxel that
 * cannot be entered.
 *
 * The route through voxel centers is then shortened into straight segments: a segment replaces a stretch of
 * the route when it crosses no voxel the route could not enter and costs no more than the stretch. The
 * points begin at exactly `start` and end at exactly `goal`.
 *
 * `canceled` is polled during the search, and the search gives up after `max_planning_time`.
 */
Plan3D plan_through_voxels(
  const CostVolume & volume,
  const Eigen::Vector3d & start,
  const Eigen::Vector3d & goal,
  const PlannerConfig & config,
  const std::function<bool()> & canceled);

// --- Heading ---

enum class YawMode
{
  kInterpolate,  // turn evenly from the start's heading to the goal's along the route
  kFaceTravel,  // face along each horizontal segment, turning in place between them and at the ends
};

/**
 * @brief Add headings to a route from `points.front()` to `points.back()`, starting at `start_yaw` and ending
 * at exactly `goal_yaw`.
 *
 * With kInterpolate, yaw turns along the shortest arc in proportion to the 3D distance travelled. With
 * kFaceTravel, the vehicle faces along each segment that is no steeper than 45 degrees, so a forward-looking
 * sonar sees where it is going; steeper segments keep the previous heading. A turn in place is a pose
 * repeated with a new heading.
 *
 * @throws std::invalid_argument if `points` has fewer than two points
 */
std::vector<Pose4D> add_yaw(
  const std::vector<Eigen::Vector3d> & points, double start_yaw, double goal_yaw, YawMode mode);

}  // namespace rov_nav2_plugins
