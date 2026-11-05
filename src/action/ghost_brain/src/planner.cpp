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

// The ghost's 0.3 m radius, plus a margin for drift
constexpr double kInflationRadius = 0.38;
// How far a start or goal stuck in the inflated zone may move to reach a free cell
constexpr double kMaxSnapDistance = 1.0;

struct Direction
{
  int dx;
  int dy;
  double cost;
};

const double kDiagonal = std::sqrt(2.0);
const Direction kDirections[] = {
  {1, 0, 1.0},
  {-1, 0, 1.0},
  {0, 1, 1.0},
  {0, -1, 1.0},
  {1, 1, kDiagonal},
  {1, -1, kDiagonal},
  {-1, 1, kDiagonal},
  {-1, -1, kDiagonal},
};

// closest free cell to (col, row) within kMaxSnapDistance, or -1
int nearest_free_cell(const nav_msgs::msg::OccupancyGrid & inflated, int col, int row)
{
  const int width = inflated.info.width;
  const int height = inflated.info.height;
  const int reach = static_cast<int>(std::ceil(kMaxSnapDistance / inflated.info.resolution));
  int best = -1;
  int best_distance = std::numeric_limits<int>::max();
  for (int ny = std::max(0, row - reach); ny <= std::min(height - 1, row + reach); ++ny) {
    for (int nx = std::max(0, col - reach); nx <= std::min(width - 1, col + reach); ++nx) {
      const int distance = (nx - col) * (nx - col) + (ny - row) * (ny - row);
      if (inflated.data[ny * width + nx] == 0 && distance <= reach * reach && distance < best_distance) {
        best = ny * width + nx;
        best_distance = distance;
      }
    }
  }
  return best;
}

}  // namespace

// kInflationRadius is the default, until the yaml says otherwise
Planner::Planner()
: settings_{kInflationRadius}
{
}

void Planner::set_settings(const PlannerSettings & settings)
{
  settings_ = settings;
  // a different inflation needs a fresh inflated map
  inflated_.data.clear();
}

const nav_msgs::msg::OccupancyGrid & Planner::inflated(const nav_msgs::msg::OccupancyGrid & map) const
{
  // same map as last time -> reuse it
  if (map.info == inflated_source_.info && map.data == inflated_source_.data && !inflated_.data.empty()) {
    return inflated_;
  }
  const int width = map.info.width;
  const int height = map.info.height;
  inflated_ = map;
  const int r = static_cast<int>(std::ceil(settings_.inflation_radius / map.info.resolution));
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (map.data[y * width + x] != 0) {
        for (int ny = std::max(0, y - r); ny <= std::min(height - 1, y + r); ++ny) {
          for (int nx = std::max(0, x - r); nx <= std::min(width - 1, x + r); ++nx) {
            if ((nx - x) * (nx - x) + (ny - y) * (ny - y) <= r * r) {
              inflated_.data[ny * width + nx] = 100;
            }
          }
        }
      }
    }
  }
  inflated_source_ = map;
  return inflated_;
}

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
  const int width = map.info.width;
  const int height = map.info.height;
  const double resolution = map.info.resolution;
  const int total_cells = width * height;

  // A goal inside something can't be reached
  if (is_occupied(map, goal.x, goal.y)) {
    return {};
  }

  // Cell inflation
  const nav_msgs::msg::OccupancyGrid & inflated = this->inflated(map);


  //Setting up for A*
  const auto to_col = [&](double x) {
    const int col = static_cast<int>(std::floor((x - map.info.origin.position.x) / resolution));
    return std::clamp(col, 0, width - 1);
  };
  const auto to_row = [&](double y) {
    const int row = static_cast<int>(std::floor((y - map.info.origin.position.y) / resolution));
    return std::clamp(row, 0, height - 1);
  };

  // The closest free cell to a cell, or -1 if there is none nearby
  const auto nearest_free = [&](int col, int row) { return nearest_free_cell(inflated, col, row); };

  const int raw_start_idx = to_row(start.y) * width + to_col(start.x);
  const int raw_goal_idx = to_row(goal.y) * width + to_col(goal.x);
  const int start_idx = nearest_free(to_col(start.x), to_row(start.y));
  const int goal_idx = nearest_free(to_col(goal.x), to_row(goal.y));
  if (start_idx < 0 || goal_idx < 0) {
    return {};
  }

  //Lambda for line-of-sight check
  const auto line_is_clear = [&](const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b) {
    const double dist = std::hypot(b.x - a.x, b.y - a.y);
    const int samples = static_cast<int>(std::ceil(dist / (resolution / 2.0)));
      for (int i = 0; i <= samples; ++i) {
        const double t = samples == 0 ? 0.0 : static_cast<double>(i) / samples;
        if (is_occupied(inflated, a.x + t * (b.x - a.x), a.y + t * (b.y - a.y))) {
        return false;
        }
        }
      return true;
  };  

  const bool start_snapped = start_idx != raw_start_idx;
  const bool goal_snapped = goal_idx != raw_goal_idx;
  const int goal_col = goal_idx % width;
  const int goal_row = goal_idx / width;

  // h(x) function
  const auto heuristic = [&](int col, int row) {
    const double dx = std::abs(col - goal_col);
    const double dy = std::abs(row - goal_row);
    return std::max(dx, dy) + (kDiagonal - 1.0) * std::min(dx, dy);
  };

  // A*
  std::vector<double> g_score(total_cells, std::numeric_limits<double>::infinity());
  std::vector<int> came_from(total_cells, -1);
  std::vector<bool> finished(total_cells, false);

  using Entry = std::pair<double, int>;  // f, cell index
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;

  g_score[start_idx] = 0.0;
  open.push({heuristic(start_idx % width, start_idx / width), start_idx});

  //Main Loop
  bool found = false;
  while (!open.empty()) {
    const int current = open.top().second;
    open.pop();

    // A cell can be queued several times, and only its first (cheapest) visit counts
    if (finished[current]) {
      continue;
    }
    finished[current] = true;

    if (current == goal_idx) {
      found = true;
      break;
    }

    const int col = current % width;
    const int row = current / width;
    for (const auto & dir : kDirections) {
      const int nx = col + dir.dx;
      const int ny = row + dir.dy;
      if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
        continue;
      }
      if (inflated.data[ny * width + nx] != 0) {
        continue;
      }
      // Don't squeeze diagonally between two blocked cells
      if (dir.dx != 0 && dir.dy != 0) {
        if (inflated.data[row * width + nx] != 0 || inflated.data[ny * width + col] != 0) {
          continue;
        }
      }

      const int neighbor = ny * width + nx;
      const double new_g = g_score[current] + dir.cost;
      if (new_g < g_score[neighbor]) {
        g_score[neighbor] = new_g;
        came_from[neighbor] = current;
        open.push({new_g + heuristic(nx, ny), neighbor});
      }
    }
  }

  if (!found) {
    return {};
  }

  // Walk back from the goal to the start, then flip it around
  std::vector<int> cells;
  for (int step = goal_idx; step != -1; step = came_from[step]) {
    cells.push_back(step);
  }
  std::reverse(cells.begin(), cells.end());

  //Sending route
  Path route;
  route.push_back(start);
  const size_t last = cells.size() - 1;
  for (size_t i = 0; i <= last; ++i) {
    if ((i == 0 && !start_snapped && last != 0) || (i == last && !goal_snapped)) {
      continue;
    }
    geometry_msgs::msg::Point point;
    point.x = map.info.origin.position.x + (cells[i] % width + 0.5) * resolution;
    point.y = map.info.origin.position.y + (cells[i] / width + 0.5) * resolution;
    route.push_back(point);
  }
  if (!goal_snapped) {
    route.push_back(goal);
  }

  // Smoothing
  Path path;
  path.push_back(route.front());
  size_t anchor = 0;
  while (anchor + 1 < route.size()) {
    size_t next = route.size() - 1;
    while (next > anchor + 1 && !line_is_clear(route[anchor], route[next])) {
      --next;
    }
    path.push_back(route[next]);
    anchor = next;
  }

  // The map is flat, so change height evenly along the route
  double total_length = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    total_length += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  double travelled = 0.0;
  path.front().z = start.z;
  for (size_t i = 1; i < path.size(); ++i) {
    travelled += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    const double t = total_length > 0.0 ? travelled / total_length : 1.0;
    path[i].z = start.z + t * (goal.z - start.z);
  }

  return path;
}

std::vector<double> Planner::distances_from(
  const geometry_msgs::msg::Point & start,
  const nav_msgs::msg::OccupancyGrid & map) const
{
  const int width = map.info.width;
  const int height = map.info.height;
  const double resolution = map.info.resolution;
  const int total_cells = width * height;
  const nav_msgs::msg::OccupancyGrid & inflated = this->inflated(map);

  std::vector<double> dist_map(total_cells, std::numeric_limits<double>::infinity());
  const int start_col = std::clamp(
    static_cast<int>(std::floor((start.x - map.info.origin.position.x) / resolution)), 0, width - 1);
  const int start_row = std::clamp(
    static_cast<int>(std::floor((start.y - map.info.origin.position.y) / resolution)), 0, height - 1);
  const int start_idx = nearest_free_cell(inflated, start_col, start_row);
  if (start_idx < 0) {
    return dist_map;
  }

  // 2. Dijkstra Flood
  using Entry = std::pair<double, int>;  // distance, cell index
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;

  dist_map[start_idx] = 0.0;
  open.push({0.0, start_idx});

  while (!open.empty()) {
    auto [d, current] = open.top();
    open.pop();

    if (d > dist_map[current]) continue;

    const int col = current % width;
    const int row = current / width;
    for (const auto & dir : kDirections) {
      const int nx = col + dir.dx;
      const int ny = row + dir.dy;
      if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
      if (inflated.data[ny * width + nx] != 0) continue;
      if (dir.dx != 0 && dir.dy != 0) {
        if (inflated.data[row * width + nx] != 0 || inflated.data[ny * width + col] != 0) continue;
      }

      const int neighbor = ny * width + nx;
      const double new_dist = dist_map[current] + dir.cost * resolution;
      if (new_dist < dist_map[neighbor]) {
        dist_map[neighbor] = new_dist;
        open.push({new_dist, neighbor});
      }
    }
  }
  return dist_map;
}

double distance_at(
  const std::vector<double> & distances,
  const nav_msgs::msg::OccupancyGrid & map,
  const geometry_msgs::msg::Point & point)
{
  const double inf = std::numeric_limits<double>::infinity();
  // plan() won't go to a point inside something, so neither do we
  if (distances.empty() || is_occupied(map, point.x, point.y)) {
    return inf;
  }
  const int width = map.info.width;
  const int height = map.info.height;
  const double resolution = map.info.resolution;
  const int col = static_cast<int>(std::floor((point.x - map.info.origin.position.x) / resolution));
  const int row = static_cast<int>(std::floor((point.y - map.info.origin.position.y) / resolution));

  // near a wall the cell itself is in the inflation, so look around for the closest free one
  const int reach = static_cast<int>(std::ceil(kMaxSnapDistance / resolution));
  double best = inf;
  for (int ny = std::max(0, row - reach); ny <= std::min(height - 1, row + reach); ++ny) {
    for (int nx = std::max(0, col - reach); nx <= std::min(width - 1, col + reach); ++nx) {
      const double step = std::hypot(nx - col, ny - row) * resolution;
      if (step <= kMaxSnapDistance) {
        best = std::min(best, distances[ny * width + nx] + step);
      }
    }
  }
  return best;
}

}  // namespace ghost_brain
