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

#include "ghost_brain/planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace ghost_brain
{

bool is_occupied(const nav_msgs::msg::OccupancyGrid & map, double x, double y)
{
  const double resolution = map.info.resolution;
  const auto col = static_cast<int64_t>(std::floor((x - map.info.origin.position.x) / resolution));
  const auto row = static_cast<int64_t>(std::floor((y - map.info.origin.position.y) / resolution));
  if (col < 0 || row < 0 || col >= map.info.width || row >= map.info.height) {
    return true;
  }
  return map.data[row * map.info.width + col] != 0;
}

namespace
{

// Ghost dimensions and vertical resolution
constexpr double kGhostRadius = 0.42;   // m, sphere radius for collisions
constexpr double kComfortRadius = 0.62;  // m, cells closer than this to anything cost extra to cross
constexpr double kComfortPenalty = 3.0;  // such a step costs this many times its length
constexpr double kResZ = 0.2;          // m, height of each 3D voxel layer
constexpr double kMinZ = 0.3;          // m, lowest ghost center height (floor clearance)
constexpr double kSnapRadius = 0.6;    // m, how far a goal in the inflation margin may be moved
constexpr double kMaxZ = 2.7;          // m, highest ghost center height (3.0m ceiling - 0.3m)

/// A* node representing one 3D voxel cell
struct Node3D
{
  int cx = 0;
  int cy = 0;
  int cz = 0;
  int index = 0;        // 1D flattened index: (cz * height + cy) * width + cx
  double g_cost = 0.0;  // cost from start to here
  double f_cost = 0.0;  // g_cost + heuristic to goal

  // Min-heap comparator for std::priority_queue (lowest f_cost popped first)
  bool operator>(const Node3D & other) const
  {
    return f_cost > other.f_cost;
  }
};

/// @return 1D flattened index from 3D grid coordinates
int to_index(int cx, int cy, int cz, int width, int height)
{
  return (cz * height + cy) * width + cx;
}

/// Convert world coordinates (x, y, z) into 3D voxel coordinates (cx, cy, cz)
void world_to_grid(
  double x, double y, double z,
  const nav_msgs::msg::OccupancyGrid & map,
  int & cx, int & cy, int & cz)
{
  cx = static_cast<int>(std::floor((x - map.info.origin.position.x) / map.info.resolution));
  cy = static_cast<int>(std::floor((y - map.info.origin.position.y) / map.info.resolution));
  cz = static_cast<int>(std::floor(z / kResZ));
}

/// Convert 3D voxel coordinates (cx, cy, cz) to world point (center of voxel)
geometry_msgs::msg::Point grid_to_world(
  int cx, int cy, int cz,
  const nav_msgs::msg::OccupancyGrid & map)
{
  geometry_msgs::msg::Point p;
  p.x = map.info.origin.position.x + (cx + 0.5) * map.info.resolution;
  p.y = map.info.origin.position.y + (cy + 0.5) * map.info.resolution;
  p.z = (cz + 0.5) * kResZ;
  return p;
}

/// @return Euclidean distance heuristic between two 3D grid cells in meters
double heuristic(int c1x, int c1y, int c1z, int c2x, int c2y, int c2z, double res_xy)
{
  double dx = (c1x - c2x) * res_xy;
  double dy = (c1y - c2y) * res_xy;
  double dz = (c1z - c2z) * kResZ;
  return std::hypot(dx, dy, dz);
}

/// Build a 3D grid (flat, indexed with to_index) where 1 = blocked, from the boxes grown by radius_m.
/// radius_m = 0 gives the "raw" grid (is the point truly inside something?),
/// radius_m = kGhostRadius gives the "inflated" grid (can the ghost's center be here?).
std::vector<char> build_cm_3d(
  const nav_msgs::msg::OccupancyGrid & map,
  const haunted_house_msgs::msg::BoxArray & obstacles,
  int depth,
  double radius_m)
{
  const int width = map.info.width;
  const int height = map.info.height;
  std::vector<char> blocked(static_cast<size_t>(width) * height * depth, 0);

  for (const auto & box : obstacles.boxes) {
    // Grow the box by radius_m in meters, then convert both corners to voxel indices
    int min_cx, min_cy, min_cz;
    int max_cx, max_cy, max_cz;
    world_to_grid(
      box.min.x - radius_m, box.min.y - radius_m, box.min.z - radius_m,
      map, min_cx, min_cy, min_cz);
    world_to_grid(
      box.max.x + radius_m, box.max.y + radius_m, box.max.z + radius_m,
      map, max_cx, max_cy, max_cz);

    // Clamp to the grid so we never write out of bounds
    min_cx = std::max(min_cx, 0);
    min_cy = std::max(min_cy, 0);
    min_cz = std::max(min_cz, 0);
    max_cx = std::min(max_cx, width - 1);
    max_cy = std::min(max_cy, height - 1);
    max_cz = std::min(max_cz, depth - 1);

    for (int cz = min_cz; cz <= max_cz; ++cz) {
      for (int cy = min_cy; cy <= max_cy; ++cy) {
        for (int cx = min_cx; cx <= max_cx; ++cx) {
          blocked[to_index(cx, cy, cz, width, height)] = 1;
        }
      }
    }
  }

  // Inflated grid only: the ghost's center can't be too close to the floor or ceiling
  if (radius_m > 0.0) {
    for (int cz = 0; cz < depth; ++cz) {
      const double z = (cz + 0.5) * kResZ;
      if (z < kMinZ || z > kMaxZ) {
        for (int cy = 0; cy < height; ++cy) {
          for (int cx = 0; cx < width; ++cx) {
            blocked[to_index(cx, cy, cz, width, height)] = 1;
          }
        }
      }
    }
  }

  return blocked;
}

/// @return true if voxel (cx, cy, cz) is outside the grid or marked blocked in `blocked`
bool is_blocked(
  int cx, int cy, int cz,
  int width, int height, int depth,
  const std::vector<char> & blocked)
{
  if (cx < 0 || cx >= width || cy < 0 || cy >= height || cz < 0 || cz >= depth) {
    return true;
  }
  return blocked[to_index(cx, cy, cz, width, height)] != 0;
}

}  // namespace

Path Planner::plan(
  const geometry_msgs::msg::Point & start,
  const geometry_msgs::msg::Point & goal,
  const nav_msgs::msg::OccupancyGrid & map,
  const haunted_house_msgs::msg::BoxArray & obstacles) const
{
  // ---------------------------------------------------------------------------
  // 1. Grid setup
  // ---------------------------------------------------------------------------
  const int width = map.info.width;
  const int height = map.info.height;
  const double res_xy = map.info.resolution;
  const int depth = static_cast<int>(std::ceil(3.0 / kResZ));  // 3.0 m tall house
  const int total_cells = width * height * depth;
  const auto inflated_cm = build_cm_3d(map, obstacles, depth, kGhostRadius);
  const auto comfort_cm = build_cm_3d(map, obstacles, depth, kComfortRadius);

  // ---------------------------------------------------------------------------
  // 2. Convert start and goal to 3D grid coordinates
  // ---------------------------------------------------------------------------
  int start_cx = 0, start_cy = 0, start_cz = 0;
  int goal_cx = 0, goal_cy = 0, goal_cz = 0;
  world_to_grid(start.x, start.y, start.z, map, start_cx, start_cy, start_cz);
  world_to_grid(goal.x, goal.y, goal.z, map, goal_cx, goal_cy, goal_cz);

  // ---------------------------------------------------------------------------
  // 3. Validation and goal snapping
  // ---------------------------------------------------------------------------
  // Start must be on the grid
  if (start_cx < 0 || start_cx >= width || start_cy < 0 || start_cy >= height ||
    start_cz < 0 || start_cz >= depth)
  {
    return {};
  }

  // A goal in the inflation margin, or even in a box's voxel (a candy right next to furniture, or
  // an estimate that has drifted a little), would never be reached. So snap it to the nearest
  // inflated-free voxel within kSnapRadius. The catch radius is 0.5 m, so ending close still counts.
  // Moves (cx, cy, cz) to the nearest inflated-free voxel within kSnapRadius. False if there is none.
  const auto snap_to_free = [&](int & cx, int & cy, int & cz) {
    if (!is_blocked(cx, cy, cz, width, height, depth, inflated_cm)) {
      return true;
    }
    const int snap_xy = static_cast<int>(std::ceil(kSnapRadius / res_xy));
    const int snap_z = static_cast<int>(std::ceil(kSnapRadius / kResZ));
    double best = std::numeric_limits<double>::infinity();
    int best_cx = cx, best_cy = cy, best_cz = cz;
    for (int dz = -snap_z; dz <= snap_z; ++dz) {
      for (int dy = -snap_xy; dy <= snap_xy; ++dy) {
        for (int dx = -snap_xy; dx <= snap_xy; ++dx) {
          if (is_blocked(cx + dx, cy + dy, cz + dz, width, height, depth, inflated_cm)) continue;
          const double d = heuristic(cx + dx, cy + dy, cz + dz, cx, cy, cz, res_xy);
          if (d <= kSnapRadius && d < best) {
            best = d;
            best_cx = cx + dx;
            best_cy = cy + dy;
            best_cz = cz + dz;
          }
        }
      }
    }
    if (best == std::numeric_limits<double>::infinity()) {
      return false;
    }
    cx = best_cx;
    cy = best_cy;
    cz = best_cz;
    return true;
  };

  // A goal in the inflation margin, or even in a box's voxel (a candy right next to furniture, or
  // an estimate that has drifted a little), would never be reached. So move it to the nearest free
  // voxel. The catch radius is 0.5 m, so ending close still counts.
  // A goal off the grid is pulled onto its edge first, so the search can still head its way
  const int clamped_cx = std::clamp(goal_cx, 0, width - 1);
  const int clamped_cy = std::clamp(goal_cy, 0, height - 1);
  const int clamped_cz = std::clamp(goal_cz, 0, depth - 1);
  bool snapped = clamped_cx != goal_cx || clamped_cy != goal_cy || clamped_cz != goal_cz ||
    is_blocked(clamped_cx, clamped_cy, clamped_cz, width, height, depth, inflated_cm);
  goal_cx = clamped_cx;
  goal_cy = clamped_cy;
  goal_cz = clamped_cz;
  if (!snap_to_free(goal_cx, goal_cy, goal_cz)) {
    // Nothing free near the goal (it is out of reach): the search below heads for the nearest
    // reachable point instead of giving up
    snapped = true;
  }

  // The same for the start: a ghost pressed against a wall (drift, a bump) is inside the margin, and
  // every neighbour would be blocked. Plan from the nearest free voxel; the route is pinned to the
  // ghost's real position below, so it flies straight out of the margin first.
  if (!snap_to_free(start_cx, start_cy, start_cz)) {
    return {};
  }

  // ---------------------------------------------------------------------------
  // 4. A* Data Structures
  // ---------------------------------------------------------------------------
  std::priority_queue<Node3D, std::vector<Node3D>, std::greater<Node3D>> open_set;
  std::vector<double> g_cost(total_cells, std::numeric_limits<double>::infinity());
  std::vector<int> came_from(total_cells, -1);

  const int start_idx = to_index(start_cx, start_cy, start_cz, width, height);
  int goal_idx = to_index(goal_cx, goal_cy, goal_cz, width, height);

  if (start_idx == goal_idx) {
    return {start, goal};
  }

  g_cost[start_idx] = 0.0;
  open_set.push(Node3D{
    start_cx, start_cy, start_cz, start_idx,
    0.0,
    heuristic(start_cx, start_cy, start_cz, goal_cx, goal_cy, goal_cz, res_xy)
  });

  bool reached_goal = false;

  // The closest to the goal the search has got, in case the goal itself cannot be reached. And a
  // limit on the work: a goal walled off from the ghost would make it search the whole house
  int closest_idx = start_idx;
  double closest_distance = heuristic(start_cx, start_cy, start_cz, goal_cx, goal_cy, goal_cz, res_xy);
  int expansions = 0;
  constexpr int kMaxExpansions = 150000;

  // ---------------------------------------------------------------------------
  // 5. A* Search Loop
  // ---------------------------------------------------------------------------
  while (!open_set.empty()) {
    Node3D node = open_set.top();
    open_set.pop();
    if(node.g_cost > g_cost[node.index]) continue;
    if (++expansions > kMaxExpansions) break;
    const double to_goal = heuristic(node.cx, node.cy, node.cz, goal_cx, goal_cy, goal_cz, res_xy);
    if (to_goal < closest_distance) {
      closest_distance = to_goal;
      closest_idx = node.index;
    }
    if(node.index == goal_idx){
      reached_goal = true;
      break;
    }

    for (int dz = -1; dz <= 1; ++dz) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0 && dz == 0) continue;

          const int ncx = node.cx + dx;
          const int ncy = node.cy + dy;
          const int ncz = node.cz + dz;

          if (is_blocked(ncx, ncy, ncz, width, height, depth, inflated_cm)) continue;

          double step_cost = std::hypot(dx * res_xy, dy * res_xy, dz * kResZ);
          if (is_blocked(ncx, ncy, ncz, width, height, depth, comfort_cm)) {
            step_cost *= kComfortPenalty;
          }
          const double tentative_g = node.g_cost + step_cost;
          const int neighbour_idx = to_index(ncx, ncy, ncz, width, height);

          if (tentative_g < g_cost[neighbour_idx]) {
            g_cost[neighbour_idx] = tentative_g;
            came_from[neighbour_idx] = node.index;
            open_set.push(Node3D{
              ncx, ncy, ncz, neighbour_idx,
              tentative_g,
              tentative_g + heuristic(ncx, ncy, ncz, goal_cx, goal_cy, goal_cz, res_xy)
            });
          }
        }
      }
    }
  }

  // If goal not reachable, return empty path
  if (!reached_goal) {
    // The goal is out of reach: go to the nearest point that is reachable instead
    if (closest_idx == start_idx) {
      return {};  // not a step closer to be had
    }
    goal_idx = closest_idx;
    snapped = true;
  }

  // ---------------------------------------------------------------------------
  // 6. Path Reconstruction
  // ---------------------------------------------------------------------------
  Path path;
  // Walk backwards from the goal; the start is the only cell with came_from == -1
  for (int idx = goal_idx; idx != -1; idx = came_from[idx]) {
    const int cx = idx % width;
    const int cy = (idx / width) % height;
    const int cz = idx / (width * height);
    path.push_back(grid_to_world(cx, cy, cz, map));
  }

  // We collected goal -> start, so flip it
  std::reverse(path.begin(), path.end());

  // Pin the ends to the exact requested points instead of voxel centers
  path.front() = start;
  if (!snapped) {
    path.back() = goal;
  }

  return path;
}

}  // namespace ghost_brain
