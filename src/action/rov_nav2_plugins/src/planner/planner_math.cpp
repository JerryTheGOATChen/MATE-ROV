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
#include "rov_nav2_plugins/planner/planner_math.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <vector>

#include "angles/angles.h"

namespace rov_nav2_plugins
{
namespace
{

constexpr double kInfinity = std::numeric_limits<double>::infinity();
constexpr std::size_t kNoParent = std::numeric_limits<std::size_t>::max();
constexpr std::size_t kPollInterval = 1024;  // expansions between checks for cancellation and the time limit
constexpr double kMaxCost = kInscribedCost - 1;  // the highest cost the vehicle can be at without touching
constexpr double kSamplesPerVoxel = 4.0;  // along a segment, when integrating its cost

/// Whether the route can step from a voxel of `from_cost` into one of `to_cost`.
bool enterable(std::uint8_t to_cost, std::uint8_t from_cost)
{
  // A vehicle in inscribed water may keep moving through it, to get out
  return to_cost < kInscribedCost || (to_cost == kInscribedCost && from_cost >= kInscribedCost);
}

/// The volume's costs as the search pays them.
class RouteCosts
{
public:
  RouteCosts(const CostVolume & volume, const PlannerConfig & config)
  : volume_(volume)
  , config_(config)
  {}

  /// Length of `offset` (m), with depth change weighted.
  double weighted_length(const Eigen::Vector3d & offset) const
  {
    return std::hypot(offset.x(), offset.y(), config_.z_weight * offset.z());
  }

  /// Cost per weighted meter in `voxel`, which must be in the grid.
  double multiplier(const Eigen::Vector3i & voxel) const
  {
    std::uint8_t cost = volume_.cost(voxel);
    if (!volume_.known(voxel)) {
      cost = std::max(cost, config_.unknown_cost);
    }
    return 1.0 + config_.cost_weight * cost / kMaxCost;
  }

  /// Cost of the straight segment from `from` to `to`, integrated along it, or infinity if it leaves the grid.
  double segment_cost(const Eigen::Vector3d & from, const Eigen::Vector3d & to) const
  {
    const GridGeometry & geometry = volume_.geometry();
    const Eigen::Vector3d offset = to - from;
    const int samples =
      std::max(1, static_cast<int>(std::ceil(offset.norm() / geometry.resolution * kSamplesPerVoxel)));
    const double sample_length = weighted_length(offset) / samples;
    double cost = 0.0;
    for (int n = 0; n < samples; ++n) {
      const auto voxel = geometry.voxel_at(from + (n + 0.5) / samples * offset);
      if (!voxel) {
        return kInfinity;
      }
      cost += sample_length * multiplier(*voxel);
    }
    return cost;
  }

  /// Whether the straight segment from `from` to `to` crosses only voxels the route could enter in turn.
  bool segment_enterable(const Eigen::Vector3d & from, const Eigen::Vector3d & to) const
  {
    std::uint8_t previous = kFreeCost;
    bool first = true;
    for (const auto & voxel : traverse(volume_.geometry(), from, to)) {
      const std::uint8_t cost = volume_.cost(voxel);
      if (!first && !enterable(cost, previous)) {
        return false;
      }
      previous = cost;
      first = false;
    }
    return true;
  }

private:
  const CostVolume & volume_;
  const PlannerConfig & config_;
};

struct Neighbor
{
  Eigen::Vector3i offset;
  double length;  // m, with depth change weighted
  // Offsets of the voxels the straight step between voxel centers can pass through, besides its ends: the
  // voxels sharing the edge or corner it cuts across
  std::vector<Eigen::Vector3i> between;
};

std::vector<Neighbor> neighbors(const RouteCosts & costs, double resolution)
{
  std::vector<Neighbor> result;
  for (int dz = -1; dz <= 1; ++dz) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0 && dz == 0) {
          continue;
        }
        const Eigen::Vector3i offset{dx, dy, dz};
        Neighbor neighbor{offset, resolution * costs.weighted_length(offset.cast<double>()), {}};
        // Every offset taking each axis either as the step does or not at all, except none and all
        for (int mask = 1; mask < 7; ++mask) {
          const Eigen::Vector3i part{mask & 1 ? dx : 0, mask & 2 ? dy : 0, mask & 4 ? dz : 0};
          if (
            part != Eigen::Vector3i::Zero() && part != offset &&
            std::find(neighbor.between.begin(), neighbor.between.end(), part) == neighbor.between.end())
          {
            neighbor.between.push_back(part);
          }
        }
        result.push_back(neighbor);
      }
    }
  }
  return result;
}

Eigen::Vector3i voxel_of(const GridGeometry & geometry, std::size_t index)
{
  const auto size_x = static_cast<std::size_t>(geometry.size.x());
  const auto size_y = static_cast<std::size_t>(geometry.size.y());
  return {
    static_cast<int>(index % size_x),
    static_cast<int>(index / size_x % size_y),
    static_cast<int>(index / (size_x * size_y))};
}

/// Shorten a route through `points` into straight segments, keeping its ends.
std::vector<Eigen::Vector3d> shorten(const std::vector<Eigen::Vector3d> & points, const RouteCosts & costs)
{
  // Cost of the route from its start to each point, measured the same way as the segments that replace it
  std::vector<double> cost_to{0.0};
  for (std::size_t i = 1; i < points.size(); ++i) {
    cost_to.push_back(cost_to.back() + costs.segment_cost(points[i - 1], points[i]));
  }

  std::vector<Eigen::Vector3d> shortened{points.front()};
  std::size_t anchor = 0;
  while (anchor + 1 < points.size()) {
    // Reach as far along the route as a straight segment can
    std::size_t reach = anchor + 1;
    while (reach + 1 < points.size() && costs.segment_enterable(points[anchor], points[reach + 1]) &&
           costs.segment_cost(points[anchor], points[reach + 1]) <= cost_to[reach + 1] - cost_to[anchor])
    {
      ++reach;
    }
    shortened.push_back(points[reach]);
    anchor = reach;
  }
  return shortened;
}

double polyline_length(const std::vector<Eigen::Vector3d> & points)
{
  double length = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    length += (points[i] - points[i - 1]).norm();
  }
  return length;
}

}  // namespace

Plan3D plan_through_voxels(
  const CostVolume & volume,
  const Eigen::Vector3d & start,
  const Eigen::Vector3d & goal,
  const PlannerConfig & config,
  const std::function<bool()> & canceled)
{
  Plan3D plan;
  const GridGeometry & geometry = volume.geometry();
  const auto start_voxel = geometry.voxel_at(start);
  const auto goal_voxel = geometry.voxel_at(goal);
  if (!start_voxel) {
    plan.status = PlanStatus::kStartOutside;
    return plan;
  }
  if (!goal_voxel) {
    plan.status = PlanStatus::kGoalOutside;
    return plan;
  }
  if (volume.cost(*start_voxel) >= kLethalCost) {
    plan.status = PlanStatus::kStartOccupied;
    return plan;
  }
  if (volume.cost(*goal_voxel) >= kInscribedCost) {
    plan.status = PlanStatus::kGoalOccupied;
    return plan;
  }

  const RouteCosts costs(volume, config);
  const std::vector<Neighbor> steps = neighbors(costs, geometry.resolution);
  const auto heuristic = [&](const Eigen::Vector3i & voxel) {
    return geometry.resolution * costs.weighted_length((*goal_voxel - voxel).cast<double>());
  };

  struct Open
  {
    double f;  // cost so far plus the heuristic
    double h;  // the heuristic, breaking ties toward the goal
    std::size_t index;
  };

  const auto later = [](const Open & a, const Open & b) { return a.f > b.f || (a.f == b.f && a.h > b.h); };
  std::priority_queue<Open, std::vector<Open>, decltype(later)> open(later);

  std::vector<double> cost_so_far(geometry.count(), kInfinity);
  std::vector<std::size_t> parent(geometry.count(), kNoParent);
  std::vector<bool> closed(geometry.count(), false);
  const std::size_t start_index = geometry.index(*start_voxel);
  const std::size_t goal_index = geometry.index(*goal_voxel);
  cost_so_far[start_index] = 0.0;
  open.push({heuristic(*start_voxel), heuristic(*start_voxel), start_index});

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(config.max_planning_time);
  bool reached = false;
  while (!open.empty()) {
    const Open top = open.top();
    open.pop();
    if (closed[top.index]) {
      continue;
    }
    closed[top.index] = true;
    if (top.index == goal_index) {
      reached = true;
      break;
    }
    if (plan.expansions++ % kPollInterval == 0) {
      if (canceled && canceled()) {
        plan.status = PlanStatus::kCanceled;
        return plan;
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        plan.status = PlanStatus::kTimedOut;
        return plan;
      }
    }

    const Eigen::Vector3i voxel = voxel_of(geometry, top.index);
    const std::uint8_t voxel_cost = volume.cost(voxel);
    for (const Neighbor & step : steps) {
      const Eigen::Vector3i next = voxel + step.offset;
      if (!geometry.contains(next)) {
        continue;
      }
      const std::size_t next_index = geometry.index(next);
      if (closed[next_index] || !enterable(volume.cost(next), voxel_cost)) {
        continue;
      }
      // A diagonal step may not cut past the corner of a voxel the route could not enter
      const auto cuts_corner = [&](const Eigen::Vector3i & part) {
        return !enterable(volume.cost(voxel + part), voxel_cost);
      };
      if (std::any_of(step.between.begin(), step.between.end(), cuts_corner)) {
        continue;
      }
      const double next_cost = cost_so_far[top.index] + step.length * costs.multiplier(next);
      if (next_cost < cost_so_far[next_index]) {
        cost_so_far[next_index] = next_cost;
        parent[next_index] = top.index;
        const double h = heuristic(next);
        open.push({next_cost + h, h, next_index});
      }
    }
  }
  if (!reached) {
    plan.status = PlanStatus::kNoPath;
    return plan;
  }

  // The route through voxel centers, from exactly the start to exactly the goal. Each end joins the center of
  // its own voxel, so no segment of the route leaves the voxels the search checked.
  std::vector<Eigen::Vector3d> points{goal};
  for (std::size_t index = goal_index; index != kNoParent; index = parent[index]) {
    points.push_back(geometry.center(voxel_of(geometry, index)));
  }
  points.push_back(start);
  std::reverse(points.begin(), points.end());
  points.erase(std::unique(points.begin(), points.end()), points.end());
  if (points.size() == 1) {
    points.push_back(goal);  // the start and the goal are the same point
  }

  plan.status = PlanStatus::kSucceeded;
  plan.search_cost = cost_so_far[goal_index];
  plan.points = shorten(points, costs);
  return plan;
}

std::vector<Pose4D> add_yaw(
  const std::vector<Eigen::Vector3d> & points, double start_yaw, double goal_yaw, YawMode mode)
{
  if (points.size() < 2) {
    throw std::invalid_argument("A route needs at least two points");
  }

  std::vector<Pose4D> path;
  if (mode == YawMode::kInterpolate) {
    const double length = polyline_length(points);
    const double yaw_change = angles::shortest_angular_distance(start_yaw, goal_yaw);
    path.push_back({points.front(), start_yaw});
    double travelled = 0.0;
    for (std::size_t i = 1; i < points.size(); ++i) {
      travelled += (points[i] - points[i - 1]).norm();
      const double fraction = length > 0.0 ? travelled / length : 1.0;
      path.push_back({points[i], angles::normalize_angle(start_yaw + yaw_change * fraction)});
    }
    path.back().yaw = goal_yaw;
    return path;
  }

  // Face along each segment, turning in place where the heading changes
  const auto append = [&path](const Eigen::Vector3d & position, double yaw) {
    if (path.empty() || path.back().position != position || path.back().yaw != yaw) {
      path.push_back({position, yaw});
    }
  };
  double heading = start_yaw;
  append(points.front(), heading);
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const Eigen::Vector3d step = points[i + 1] - points[i];
    const double horizontal = step.head<2>().norm();
    if (horizontal > 0.0 && horizontal >= std::abs(step.z())) {
      heading = std::atan2(step.y(), step.x());
      append(points[i], heading);
    }
    append(points[i + 1], heading);
  }
  append(points.back(), goal_yaw);
  return path;
}

}  // namespace rov_nav2_plugins
