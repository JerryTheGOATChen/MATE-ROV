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
#include "ghost_brain/controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace ghost_brain
{

namespace
{

constexpr double kGridResolution = 0.2;
constexpr double kGhostRadius = 0.31;
constexpr double kSamplingInterval = 0.05;
constexpr double kFloorHeight = 0.0;
constexpr double kCeilingHeight = 3.0;
constexpr double kApproachDistance = 0.40;  // Collection radius is 0.5 m.
constexpr double kApproachMargin = 0.01;
constexpr double kEpsilon = 1e-9;

struct GridPosition
{
  int x;
  int y;
  int z;
};

GridPosition world_to_grid(
  const geometry_msgs::msg::Point & point,
  const geometry_msgs::msg::Point & origin)
{
  return {
    static_cast<int>(std::floor((point.x - origin.x) / kGridResolution)),
    static_cast<int>(std::floor((point.y - origin.y) / kGridResolution)),
    static_cast<int>(std::floor((point.z - origin.z) / kGridResolution))};
}

geometry_msgs::msg::Point grid_to_world(
  const GridPosition & grid,
  const geometry_msgs::msg::Point & origin)
{
  geometry_msgs::msg::Point point;
  point.x = origin.x + (grid.x + 0.5) * kGridResolution;
  point.y = origin.y + (grid.y + 0.5) * kGridResolution;
  point.z = origin.z + (grid.z + 0.5) * kGridResolution;
  return point;
}

bool in_bounds(const GridPosition & p, int nx, int ny, int nz)
{
  return p.x >= 0 && p.x < nx && p.y >= 0 && p.y < ny && p.z >= 0 && p.z < nz;
}

int grid_to_index(const GridPosition & p, int nx, int ny)
{
  return (p.z * ny + p.y) * nx + p.x;
}

GridPosition index_to_grid(int index, int nx, int ny)
{
  return {index % nx, (index / nx) % ny, index / (nx * ny)};
}

double distance(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  return std::hypot(a.x - b.x, a.y - b.y, a.z - b.z);
}

bool finite_point(const geometry_msgs::msg::Point & p)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

// A sphere is conservatively treated as a point against boxes enlarged by its radius.
bool is_blocked(
  const geometry_msgs::msg::Point & point,
  const haunted_house_msgs::msg::BoxArray & obstacles)
{
  for (const auto & box : obstacles.boxes) {
    if (point.x >= box.min.x - kGhostRadius &&
        point.x <= box.max.x + kGhostRadius &&
        point.y >= box.min.y - kGhostRadius &&
        point.y <= box.max.y + kGhostRadius &&
        point.z >= box.min.z - kGhostRadius &&
        point.z <= box.max.z + kGhostRadius) {
      return true;
    }
  }
  return false;
}

// Constrain an interval of segment parameter t (0..1) to one inflated box axis.
bool overlaps_axis(
  double a, double b, double low, double high,
  double & enter, double & leave)
{
  const double delta = b - a;
  if (std::abs(delta) < 1e-12) {
    return a >= low && a <= high;
  }

  double t0 = (low - a) / delta;
  double t1 = (high - a) / delta;
  if (t0 > t1) {
    std::swap(t0, t1);
  }
  enter = std::max(enter, t0);
  leave = std::min(leave, t1);
  return enter <= leave;
}

// Exact line-segment vs. inflated axis-aligned box test: no sampling gaps.
bool segment_is_clear(
  const geometry_msgs::msg::Point & start,
  const geometry_msgs::msg::Point & end,
  const haunted_house_msgs::msg::BoxArray & obstacles)
{
  for (const auto & box : obstacles.boxes) {
    double enter = 0.0;
    double leave = 1.0;
    if (overlaps_axis(start.x, end.x, box.min.x - kGhostRadius,
                      box.max.x + kGhostRadius, enter, leave) &&
        overlaps_axis(start.y, end.y, box.min.y - kGhostRadius,
                      box.max.y + kGhostRadius, enter, leave) &&
        overlaps_axis(start.z, end.z, box.min.z - kGhostRadius,
                      box.max.z + kGhostRadius, enter, leave)) {
      return false;
    }
  }
  return true;
}

struct SearchSpace
{
  const nav_msgs::msg::OccupancyGrid & map;
  const haunted_house_msgs::msg::BoxArray * obstacles;  // nullptr: 2D fallback.
  double min_x;
  double max_x;
  double min_y;
  double max_y;

  bool point_is_clear(const geometry_msgs::msg::Point & p) const
  {
    // The whole sphere must remain inside the house footprint.
    if (!finite_point(p) || p.x < min_x + kGhostRadius - kEpsilon ||
        p.x > max_x - kGhostRadius + kEpsilon ||
        p.y < min_y + kGhostRadius - kEpsilon ||
        p.y > max_y - kGhostRadius + kEpsilon) {
      return false;
    }

    if (obstacles != nullptr) {
      if (p.z < kFloorHeight + kGhostRadius - kEpsilon ||
          p.z > kCeilingHeight - kGhostRadius + kEpsilon) {
        return false;
      }
      return !is_blocked(p, *obstacles);
    }

    // Legacy 2D planner: check all occupied map cells near the ghost's footprint.
    const double resolution = map.info.resolution;
    const int first_x = static_cast<int>(std::floor((p.x - kGhostRadius - min_x) / resolution));
    const int last_x = static_cast<int>(std::floor((p.x + kGhostRadius - min_x) / resolution));
    const int first_y = static_cast<int>(std::floor((p.y - kGhostRadius - min_y) / resolution));
    const int last_y = static_cast<int>(std::floor((p.y + kGhostRadius - min_y) / resolution));

    for (int y = first_y; y <= last_y; ++y) {
      for (int x = first_x; x <= last_x; ++x) {
        if (x < 0 || y < 0 || x >= static_cast<int>(map.info.width) ||
            y >= static_cast<int>(map.info.height)) {
          return false;
        }
        if (map.data[static_cast<size_t>(y) * map.info.width + x] == 0) {
          continue;
        }
        const double x0 = min_x + x * resolution;
        const double y0 = min_y + y * resolution;
        const double near_x = std::clamp(p.x, x0, x0 + resolution);
        const double near_y = std::clamp(p.y, y0, y0 + resolution);
        const double dx = p.x - near_x;
        const double dy = p.y - near_y;
        if (dx * dx + dy * dy <= kGhostRadius * kGhostRadius) {
          return false;
        }
      }
    }
    return true;
  }

  bool segment_is_clear(
    const geometry_msgs::msg::Point & a,
    const geometry_msgs::msg::Point & b) const
  {
    if (!point_is_clear(a) || !point_is_clear(b)) {
      return false;
    }
    if (obstacles != nullptr) {
      // The world bounds are convex, so clear endpoints remain within them.
      return ghost_brain::segment_is_clear(a, b, *obstacles);
    }

    // The 2D fallback has no obstacle boxes; sample the inflated occupancy grid.
    const double step = std::min(kSamplingInterval, map.info.resolution / 2.0);
    const int intervals = std::max(1, static_cast<int>(std::ceil(distance(a, b) / step)));
    for (int i = 1; i < intervals; ++i) {
      const double t = static_cast<double>(i) / intervals;
      geometry_msgs::msg::Point p;
      p.x = a.x + t * (b.x - a.x);
      p.y = a.y + t * (b.y - a.y);
      p.z = a.z + t * (b.z - a.z);
      if (!point_is_clear(p)) {
        return false;
      }
    }
    return true;
  }
};

Path smooth_path(const Path & raw, const SearchSpace & space)
{
  if (raw.size() <= 2) {
    return raw;
  }
  Path result;
  result.push_back(raw.front());
  size_t anchor = 0;
  while (anchor + 1 < raw.size()) {
    size_t next = anchor + 1;
    for (size_t candidate = raw.size() - 1; candidate > anchor + 1; --candidate) {
      if (space.segment_is_clear(raw[anchor], raw[candidate])) {
        next = candidate;
        break;
      }
    }
    result.push_back(raw[next]);
    anchor = next;
  }
  return result;
}

Path run_astar(
  const geometry_msgs::msg::Point & start,
  const geometry_msgs::msg::Point & goal,
  const nav_msgs::msg::OccupancyGrid & map,
  const haunted_house_msgs::msg::BoxArray * obstacles)
{
  if (!finite_point(start) || !finite_point(goal) ||
      !std::isfinite(map.info.resolution) || map.info.resolution <= 0.0 ||
      map.info.width == 0 || map.info.height == 0 ||
      map.data.size() < static_cast<size_t>(map.info.width) * map.info.height) {
    return {};
  }

  const double width_m = map.info.width * static_cast<double>(map.info.resolution);
  const double height_m = map.info.height * static_cast<double>(map.info.resolution);
  const double nx_double = std::ceil(width_m / kGridResolution);
  const double ny_double = std::ceil(height_m / kGridResolution);
  const double nz_double = obstacles ? std::ceil((kCeilingHeight - kFloorHeight) / kGridResolution) : 1.0;
  if (!std::isfinite(nx_double) || !std::isfinite(ny_double) ||
      nx_double < 1 || ny_double < 1 ||
      nx_double * ny_double * nz_double > 3000000.0) {
    return {};
  }
  const int nx = static_cast<int>(nx_double);
  const int ny = static_cast<int>(ny_double);
  const int nz = static_cast<int>(nz_double);
  const int total_cells = nx * ny * nz;

  geometry_msgs::msg::Point origin = map.info.origin.position;
  origin.z = obstacles ? kFloorHeight : start.z - kGridResolution / 2.0;
  const SearchSpace space{
    map, obstacles,
    origin.x, origin.x + width_m,
    origin.y, origin.y + height_m};

  if (!space.point_is_clear(start)) {
    if (!obstacles) {
      return {};
    }
    // Allow only the initial recovery segment to overlap boxes containing start.
    // Exit through a nearest face so recovery does not tunnel across a wall.
    haunted_house_msgs::msg::BoxArray other_boxes;
    std::vector<const haunted_house_msgs::msg::Box *> containing;
    for (const auto & box : obstacles->boxes) {
      if (start.x >= box.min.x - kGhostRadius && start.x <= box.max.x + kGhostRadius &&
        start.y >= box.min.y - kGhostRadius && start.y <= box.max.y + kGhostRadius &&
        start.z >= box.min.z - kGhostRadius && start.z <= box.max.z + kGhostRadius)
      {
        containing.push_back(&box);
      } else {
        other_boxes.boxes.push_back(box);
      }
    }
    geometry_msgs::msg::Point escape;
    double best_cost = std::numeric_limits<double>::infinity();
    for (int dz = -1; dz <= 1; ++dz) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const double norm = std::sqrt(dx * dx + dy * dy + dz * dz);
          for (int step = 1; step <= 16; ++step) {
            const double length = step * 0.05;
            geometry_msgs::msg::Point candidate = start;
            candidate.x += length * dx / norm;
            candidate.y += length * dy / norm;
            candidate.z += length * dz / norm;
            // Controller may advance within 10 cm of a waypoint. Put the recovery
            // endpoint far enough into free space that advancing cannot cut the exit.
            auto before_candidate = candidate;
            before_candidate.x -= 0.12 * dx / norm;
            before_candidate.y -= 0.12 * dy / norm;
            before_candidate.z -= 0.12 * dz / norm;
            if (length < 0.15 || !space.point_is_clear(candidate) ||
              !space.point_is_clear(before_candidate) ||
              !ghost_brain::segment_is_clear(start, candidate, other_boxes))
            {
              continue;
            }
            bool exits_nearest = true;
            for (const auto * box : containing) {
              const double low_x = box->min.x - kGhostRadius;
              const double high_x = box->max.x + kGhostRadius;
              const double low_y = box->min.y - kGhostRadius;
              const double high_y = box->max.y + kGhostRadius;
              const double low_z = box->min.z - kGhostRadius;
              const double high_z = box->max.z + kGhostRadius;
              const double nearest = std::min({start.x - low_x, high_x - start.x,
                start.y - low_y, high_y - start.y, start.z - low_z, high_z - start.z});
              const bool exits =
                (start.x - low_x <= nearest + kEpsilon && candidate.x < low_x) ||
                (high_x - start.x <= nearest + kEpsilon && candidate.x > high_x) ||
                (start.y - low_y <= nearest + kEpsilon && candidate.y < low_y) ||
                (high_y - start.y <= nearest + kEpsilon && candidate.y > high_y) ||
                (start.z - low_z <= nearest + kEpsilon && candidate.z < low_z) ||
                (high_z - start.z <= nearest + kEpsilon && candidate.z > high_z);
              exits_nearest = exits_nearest && exits;
            }
            const double cost = length + 0.02 * distance(candidate, goal);
            if (exits_nearest && cost < best_cost) {
              best_cost = cost;
              escape = candidate;
            }
          }
        }
      }
    }
    if (!std::isfinite(best_cost)) {
      return {};
    }
    auto route = run_astar(escape, goal, map, obstacles);
    if (route.empty()) {
      return {start, escape};  // Still leave the overlap even if the goal is unreachable.
    }
    route.insert(route.begin(), start);
    return route;
  }
  // Find collision-free *endpoints*. A candy's centre might be inside an
  // inflated furniture box, even though we can collect it from nearby.
  std::vector<geometry_msgs::msg::Point> destinations;
  const bool goal_is_clear = space.point_is_clear(goal);
  if (goal_is_clear) {
    destinations.push_back(goal);
  } else if (obstacles != nullptr) {
    if (distance(start, goal) <= kApproachDistance) {
      return {start};  // Already within collection range.
    }

    auto add_approach = [&](const geometry_msgs::msg::Point & p) {
      if (distance(p, goal) <= kApproachDistance && space.point_is_clear(p)) {
        destinations.push_back(p);
      }
    };

    // For a goal on furniture, try points just outside the inflated box faces.
    for (const auto & box : obstacles->boxes) {
      geometry_msgs::msg::Point p = goal;
      p.x = box.min.x - kGhostRadius - kApproachMargin;
      add_approach(p);
      p.x = box.max.x + kGhostRadius + kApproachMargin;
      add_approach(p);
      p = goal;
      p.y = box.min.y - kGhostRadius - kApproachMargin;
      add_approach(p);
      p.y = box.max.y + kGhostRadius + kApproachMargin;
      add_approach(p);
      p = goal;
      p.z = box.min.z - kGhostRadius - kApproachMargin;
      add_approach(p);
      p.z = box.max.z + kGhostRadius + kApproachMargin;
      add_approach(p);
    }

    // Targets near the floor/ceiling or an obstacle corner may need other
    // approach directions; sample the shell around the target as a fallback.
    for (int dz = -1; dz <= 1; ++dz) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const double scale = (kApproachDistance - kApproachMargin) /
            std::sqrt(dx * dx + dy * dy + dz * dz);
          geometry_msgs::msg::Point p = goal;
          p.x += dx * scale;
          p.y += dy * scale;
          p.z += dz * scale;
          add_approach(p);
        }
      }
    }
  }
  if (destinations.empty()) {
    return {};  // No safe endpoint within collection range.
  }

  // The fast path: if a straight flight is safe, skip A* altogether.
  double best_direct_length = std::numeric_limits<double>::infinity();
  const geometry_msgs::msg::Point * direct_endpoint = nullptr;
  for (const auto & p : destinations) {
    const double length = distance(start, p);
    if (length < best_direct_length && space.segment_is_clear(start, p)) {
      best_direct_length = length;
      direct_endpoint = &p;
    }
  }
  if (direct_endpoint != nullptr) {
    return {start, *direct_endpoint};
  }

  const GridPosition start_grid = world_to_grid(start, origin);
  if (!in_bounds(start_grid, nx, ny, nz)) {
    return {};
  }

  std::vector<double> g_score(total_cells, std::numeric_limits<double>::infinity());
  std::vector<int> parent(total_cells, -1);
  std::vector<bool> closed(total_cells, false);
  using QueueEntry = std::pair<double, int>;  // Estimated total cost, cell index.
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open_set;

  auto heuristic = [&](const geometry_msgs::msg::Point & p) {
    double best = std::numeric_limits<double>::infinity();
    for (const auto & candidate : destinations) {
      const double d = obstacles ? distance(p, candidate) :
        std::hypot(p.x - candidate.x, p.y - candidate.y);
      best = std::min(best, d);
    }
    return best;
  };

  // Seed nearby free voxels connected to the ghost's *exact* start position.
  // This also handles cases where rounding places the start in a blocked voxel.
  for (int dz = -1; dz <= 1; ++dz) {
    if (!obstacles && dz != 0) {
      continue;
    }
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        const GridPosition p{start_grid.x + dx, start_grid.y + dy, start_grid.z + dz};
        if (!in_bounds(p, nx, ny, nz)) {
          continue;
        }
        const auto world = grid_to_world(p, origin);
        if (!space.segment_is_clear(start, world)) {
          continue;
        }
        const int index = grid_to_index(p, nx, ny);
        const double cost = distance(start, world);
        if (cost >= g_score[index]) {
          continue;
        }
        g_score[index] = cost;
        parent[index] = -2;  // Seed: connect straight back to the exact start.
        open_set.push({cost + heuristic(world), index});
      }
    }
  }

  int reached_index = -1;
  geometry_msgs::msg::Point reached_destination;
  while (!open_set.empty()) {
    const auto [queued_f, current_index] = open_set.top();
    open_set.pop();
    if (closed[current_index]) {
      continue;
    }
    const auto current_grid = index_to_grid(current_index, nx, ny);
    const auto current_world = grid_to_world(current_grid, origin);
    if (queued_f > g_score[current_index] + heuristic(current_world) + kEpsilon) {
      continue;  // Outdated queue entry from a worse route.
    }
    closed[current_index] = true;

    // Attach the exact safe endpoint when it is nearby and directly reachable.
    for (const auto & destination : destinations) {
      if (distance(current_world, destination) <= 0.8 &&
          space.segment_is_clear(current_world, destination)) {
        reached_index = current_index;
        reached_destination = destination;
        break;
      }
    }
    if (reached_index >= 0) {
      break;
    }

    for (int dz = -1; dz <= 1; ++dz) {
      if (!obstacles && dz != 0) {
        continue;
      }
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const GridPosition next{
            current_grid.x + dx, current_grid.y + dy, current_grid.z + dz};
          if (!in_bounds(next, nx, ny, nz)) {
            continue;
          }
          const int next_index = grid_to_index(next, nx, ny);
          if (closed[next_index]) {
            continue;
          }
          const auto next_world = grid_to_world(next, origin);
          if (!space.segment_is_clear(current_world, next_world)) {
            continue;
          }
          const double step = kGridResolution * std::sqrt(dx * dx + dy * dy + dz * dz);
          const double candidate_g = g_score[current_index] + step;
          if (candidate_g + kEpsilon >= g_score[next_index]) {
            continue;
          }
          g_score[next_index] = candidate_g;
          parent[next_index] = current_index;
          open_set.push({candidate_g + heuristic(next_world), next_index});
        }
      }
    }
  }

  if (reached_index < 0) {
    return {};
  }

  // Reconstruct the voxel chain in reverse, then attach the exact start/goal.
  Path reverse_grid_path;
  int index = reached_index;
  for (int steps = 0; steps < total_cells; ++steps) {
    reverse_grid_path.push_back(grid_to_world(index_to_grid(index, nx, ny), origin));
    if (parent[index] == -2) {
      break;
    }
    if (parent[index] < 0) {
      return {};
    }
    index = parent[index];
  }
  std::reverse(reverse_grid_path.begin(), reverse_grid_path.end());
  Path raw_path;
  raw_path.push_back(start);
  raw_path.insert(raw_path.end(), reverse_grid_path.begin(), reverse_grid_path.end());
  raw_path.push_back(reached_destination);
  return smooth_path(raw_path, space);
}

}  // namespace

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

Path Planner::plan(
  const geometry_msgs::msg::Point & start,
  const geometry_msgs::msg::Point & goal,
  const nav_msgs::msg::OccupancyGrid & map) const
{
  // Preserve the starter interface: use the 2D occupancy map when no boxes are supplied.
  return run_astar(start, goal, map, nullptr);
}

Path Planner::plan(
  const geometry_msgs::msg::Point & start,
  const geometry_msgs::msg::Point & goal,
  const nav_msgs::msg::OccupancyGrid & map,
  const haunted_house_msgs::msg::BoxArray & obstacles) const
{
  // Full 3D A* using the house's exact 3D obstacle boxes.
  return run_astar(start, goal, map, &obstacles);
}

}  // namespace ghost_brain

namespace ghost_brain
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kClearance = 0.36;

double exploration_distance(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

std::pair<int, int> cell(const geometry_msgs::msg::Point & p)
{
  return {static_cast<int>(std::floor(p.x)), static_cast<int>(std::floor(p.y))};
}

// Check the entire footprint against occupied cells, including map boundaries.
bool clear(const nav_msgs::msg::OccupancyGrid & map, double x, double y)
{
  const double r = map.info.resolution;
  const double ox = map.info.origin.position.x;
  const double oy = map.info.origin.position.y;
  for (int row = static_cast<int>(std::floor((y - kClearance - oy) / r));
    row <= static_cast<int>(std::floor((y + kClearance - oy) / r)); ++row)
  {
    for (int col = static_cast<int>(std::floor((x - kClearance - ox) / r));
      col <= static_cast<int>(std::floor((x + kClearance - ox) / r)); ++col)
    {
      const double dx = x - std::clamp(x, ox + col * r, ox + (col + 1) * r);
      const double dy = y - std::clamp(y, oy + row * r, oy + (row + 1) * r);
      if (dx * dx + dy * dy <= kClearance * kClearance &&
        is_occupied(map, ox + (col + 0.5) * r, oy + (row + 0.5) * r))
      {
        return false;
      }
    }
  }
  return true;
}

double free_distance(
  const nav_msgs::msg::OccupancyGrid & map,
  const geometry_msgs::msg::Point & here, double angle)
{
  for (double d = 0.05; d <= 2.0; d += 0.05) {
    if (!clear(map, here.x + d * std::cos(angle), here.y + d * std::sin(angle))) {
      return d - 0.05;
    }
  }
  return 2.0;
}
}  // namespace

void Explorer::interrupt()
{
  history_.clear();
  progress_.reset();
  escape_goal_.reset();
  path_.clear();
  travel_ = 0.0;
  following_wall_ = false;
}

void Explorer::reset()
{
  interrupt();
  coverage_.clear();
}

bool Explorer::start_escape(
  const geometry_msgs::msg::Point & here,
  const nav_msgs::msg::OccupancyGrid & map, double time)
{
  struct Candidate { geometry_msgs::msg::Point point; double score; };
  std::vector<Candidate> candidates;
  const double ox = map.info.origin.position.x;
  const double oy = map.info.origin.position.y;
  for (double y = oy + 0.5; y < oy + map.info.height * map.info.resolution; y += 1.0) {
    for (double x = ox + 0.5; x < ox + map.info.width * map.info.resolution; x += 1.0) {
      auto p = here;
      p.x = x;
      p.y = y;
      const double d = exploration_distance(here, p);
      if (d < 2.5 || !clear(map, x, y)) {
        continue;
      }
      // Prefer new territory, then greater separation from the loop or stall.
      unsigned visits = 0;
      for (const auto & entry : coverage_) {
        const auto key = cell(p);
        if (std::abs(entry.first.first - key.first) <= 1 &&
          std::abs(entry.first.second - key.second) <= 1)
        {
          visits += entry.second;
        }
      }
      candidates.push_back({p, 4.0 * visits - std::min(d, 8.0)});
    }
  }
  std::sort(candidates.begin(), candidates.end(),
    [](const Candidate & a, const Candidate & b) { return a.score < b.score; });
  // Bound expensive searches; failed destinations get a penalty for the next attempt.
  for (size_t i = 0; i < std::min<size_t>(12, candidates.size()); ++i) {
    const auto & goal = candidates[i].point;
    auto route = planner_.plan(here, goal, map);
    if (route.size() >= 2 && exploration_distance(route.back(), goal) < 0.1) {
      escape_goal_ = goal;
      escape_started_ = time;
      last_plan_ = time;
      path_ = std::move(route);
      history_.clear();
      following_wall_ = false;
      return true;
    }
    coverage_[cell(goal)] += 10;
  }
  return false;
}

geometry_msgs::msg::Twist Explorer::update(
  const geometry_msgs::msg::Pose & pose,
  const nav_msgs::msg::OccupancyGrid & map, double time)
{
  geometry_msgs::msg::Twist command;
  if (map.info.resolution <= 0.0 || map.info.width == 0 || map.info.height == 0 ||
    map.data.size() < static_cast<size_t>(map.info.width) * map.info.height)
  {
    path_.clear();
    return command;
  }
  const auto & here = pose.position;
  const auto & q = pose.orientation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  if (progress_ && time < progress_->time) {
    interrupt();
  }
  if (!progress_) {
    progress_ = Visit{here, yaw, time, 0.0};
    ++coverage_[cell(here)];
  }
  bool loop = false;
  if (exploration_distance(here, progress_->position) >= 0.25) {
    travel_ += exploration_distance(here, progress_->position);
    progress_ = Visit{here, yaw, time, travel_};
    ++coverage_[cell(here)];
    for (const auto & visit : history_) {
      if (time - visit.time >= 8.0 && travel_ - visit.travel >= 4.0 &&
        exploration_distance(here, visit.position) < 0.65 && std::cos(yaw - visit.yaw) > 0.5)
      {
        loop = true;
        break;
      }
    }
    history_.push_back(*progress_);
    if (history_.size() > 600) {
      history_.erase(history_.begin());
    }
  }
  const bool stalled = time - progress_->time >= 3.0;
  if (escape_goal_ && (exploration_distance(here, *escape_goal_) < 0.35 ||
    time - escape_started_ > 25.0 || stalled))
  {
    coverage_[cell(*escape_goal_)] += 10;
    escape_goal_.reset();
    history_.clear();
    path_.clear();
    following_wall_ = false;
  }
  if (!escape_goal_ && (loop || stalled)) {
    start_escape(here, map, time);
    progress_ = Visit{here, yaw, time, travel_};
  }
  if (escape_goal_) {
    if (time - last_plan_ >= 0.5) {
      path_ = planner_.plan(here, *escape_goal_, map);
      last_plan_ = time;
    }
    while (path_.size() > 2 && exploration_distance(here, path_[1]) < 0.10) {
      path_.erase(path_.begin() + 1);
    }
    return Controller{}.compute_velocity(pose, path_);
  }

  path_.clear();
  const double front = free_distance(map, here, yaw);
  const double left = free_distance(map, here, yaw + kPi / 2.0);
  if (left < 1.2) {
    following_wall_ = true;
  }
  if (front < 0.55) {
    // Turn right at an obstacle, keeping the wall on the left.
    command.angular.z = -0.8;
    return command;
  }
  // Before finding a wall, move forward until reaching one. Once attached,
  // steer toward a missing left wall to follow outside corners and doorways.
  const double turn = following_wall_ ? std::clamp(1.4 * (left - 0.30), -0.7, 0.7) : 0.0;
  auto goal = here;
  const double heading = yaw + turn;
  const double length = std::min(0.5, free_distance(map, here, heading));
  if (length < 0.15) {
    command.angular.z = -0.8;
    return command;
  }
  goal.x += length * std::cos(heading);
  goal.y += length * std::sin(heading);
  path_ = {here, goal};
  command = Controller{}.compute_velocity(pose, path_);
  // Exploration is deliberately slower than target pursuit.
  command.linear.x = std::clamp(command.linear.x, -0.45, 0.45);
  command.linear.y = std::clamp(command.linear.y, -0.3, 0.3);
  return command;
}

}  // namespace ghost_brain
