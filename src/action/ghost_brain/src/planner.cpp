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
#include <memory>
#include <map>
#include <queue>
#include <utility>

namespace ghost_brain
{
namespace
{
bool valid_map(const nav_msgs::msg::OccupancyGrid & map)
{
  return std::isfinite(map.info.resolution) && map.info.resolution > 0.0 &&
         map.info.width > 0 && map.info.height > 0 &&
         map.data.size() == static_cast<size_t>(map.info.width) * map.info.height;
}
}  // namespace

bool is_occupied(const nav_msgs::msg::OccupancyGrid & map, double x, double y)
{
  if (!valid_map(map) || !std::isfinite(x) || !std::isfinite(y)) {
    return true;
  }
  const double resolution = map.info.resolution;
  const auto col = static_cast<int64_t>(std::floor((x - map.info.origin.position.x) / resolution));
  const auto row = static_cast<int64_t>(std::floor((y - map.info.origin.position.y) / resolution));
  if (col < 0 || row < 0 || col >= map.info.width || row >= map.info.height) {
    return true;
  }
  return map.data[row * map.info.width + col] != 0;
}

bool is_occupied_with_clearance(
  const nav_msgs::msg::OccupancyGrid & map, double x, double y, double radius)
{
  if (!std::isfinite(radius) || radius < 0.0 || is_occupied(map, x, y)) {
    return true;
  }
  if (radius == 0.0) {
    return false;
  }
  const double resolution = map.info.resolution;
  const double local_x = x - map.info.origin.position.x;
  const double local_y = y - map.info.origin.position.y;
  if (local_x <= radius || local_y <= radius ||
      map.info.width * resolution - local_x <= radius ||
      map.info.height * resolution - local_y <= radius) {
    return true;
  }
  const int padding = static_cast<int>(std::ceil(radius / resolution)) + 1;
  const int col = static_cast<int>(std::floor(local_x / resolution));
  const int row = static_cast<int>(std::floor(local_y / resolution));
  for (int ny = std::max(0, row - padding); ny <= std::min(static_cast<int>(map.info.height) - 1, row + padding); ++ny) {
    for (int nx = std::max(0, col - padding); nx <= std::min(static_cast<int>(map.info.width) - 1, col + padding); ++nx) {
      if (map.data[ny * map.info.width + nx] == 0) {
        continue;
      }
      const double dx = std::max({nx * resolution - local_x, 0.0, local_x - (nx + 1) * resolution});
      const double dy = std::max({ny * resolution - local_y, 0.0, local_y - (ny + 1) * resolution});
      if (dx * dx + dy * dy <= radius * radius + 1e-12) {
        return true;
      }
    }
  }
  return false;
}

nav_msgs::msg::OccupancyGrid inflate_map(const nav_msgs::msg::OccupancyGrid & map, double radius)
{
  if (!valid_map(map) || !std::isfinite(radius) || radius < 0.0) {
    return nav_msgs::msg::OccupancyGrid{};
  }
  auto inflated = map;
  if (radius == 0.0) {
    return inflated;
  }
  const double resolution = map.info.resolution;
  const int padding = static_cast<int>(std::ceil(radius / resolution)) + 1;
  std::vector<std::pair<int, int>> offsets;
  for (int dy = -padding; dy <= padding; ++dy) {
    for (int dx = -padding; dx <= padding; ++dx) {
      // Distance from a candidate cell center to the occupied cell's square footprint.
      const double gap_x = std::max(std::abs(dx) - 0.5, 0.0) * resolution;
      const double gap_y = std::max(std::abs(dy) - 0.5, 0.0) * resolution;
      if (gap_x * gap_x + gap_y * gap_y <= radius * radius + 1e-12) {
        offsets.emplace_back(dx, dy);
      }
    }
  }
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      if ((col + 0.5) * resolution <= radius || (row + 0.5) * resolution <= radius ||
          (width - col - 0.5) * resolution <= radius || (height - row - 0.5) * resolution <= radius) {
        inflated.data[row * width + col] = 100;
      }
      if (map.data[row * width + col] == 0) {
        continue;
      }
      for (const auto & offset : offsets) {
        const int x = col + offset.first;
        const int y = row + offset.second;
        if (x >= 0 && y >= 0 && x < width && y < height) {
          inflated.data[y * width + x] = 100;
        }
      }
    }
  }
  return inflated;
}

bool is_navigation_step_valid(
  const nav_msgs::msg::OccupancyGrid & map, const nav_msgs::msg::OccupancyGrid & navigation,
  int from_col, int from_row, int to_col, int to_row)
{
  if (!valid_map(map) || navigation.info.width != map.info.width ||
      navigation.info.height != map.info.height || navigation.data.size() != map.data.size()) {
    return false;
  }
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  if (from_col < 0 || from_row < 0 || to_col < 0 || to_row < 0 ||
      from_col >= width || to_col >= width || from_row >= height || to_row >= height) {
    return false;
  }
  const int dx = to_col - from_col;
  const int dy = to_row - from_row;
  if ((dx == 0 && dy == 0) || std::abs(dx) > 1 || std::abs(dy) > 1 ||
      map.data[from_row * width + from_col] != 0 || map.data[to_row * width + to_col] != 0) {
    return false;
  }
  if (navigation.data[from_row * width + from_col] == 0) {
    return navigation.data[to_row * width + to_col] == 0 &&
           (dx == 0 || dy == 0 ||
            (navigation.data[from_row * width + to_col] == 0 &&
             navigation.data[to_row * width + from_col] == 0));
  }
  // A buffer is not a physical wall. Allow an already-buffered position to leave it,
  // using cardinal steps that maintain or improve clearance from actual obstacles.
  if (dx != 0 && dy != 0) {
    return false;
  }
  if (navigation.data[to_row * width + to_col] == 0) {
    return true;
  }
  const double resolution = map.info.resolution;
  const int padding = static_cast<int>(std::ceil(kObstacleInflationRadius / resolution)) + 1;
  auto clearance_squared = [&](int col, int row) {
    const double x = (col + 0.5) * resolution;
    const double y = (row + 0.5) * resolution;
    const double boundary = std::min({x, y, width * resolution - x, height * resolution - y});
    double nearest = std::min(boundary * boundary,
      (kObstacleInflationRadius + resolution) * (kObstacleInflationRadius + resolution));
    for (int ny = std::max(0, row - padding); ny <= std::min(height - 1, row + padding); ++ny) {
      for (int nx = std::max(0, col - padding); nx <= std::min(width - 1, col + padding); ++nx) {
        if (map.data[ny * width + nx] == 0) {
          continue;
        }
        const double gap_x = std::max({nx * resolution - x, 0.0, x - (nx + 1) * resolution});
        const double gap_y = std::max({ny * resolution - y, 0.0, y - (ny + 1) * resolution});
        nearest = std::min(nearest, gap_x * gap_x + gap_y * gap_y);
      }
    }
    return nearest;
  };
  return clearance_squared(to_col, to_row) + 1e-12 >= clearance_squared(from_col, from_row);
}

Path Planner::plan(
  const geometry_msgs::msg::Point & start,
  const geometry_msgs::msg::Point & goal,
  const nav_msgs::msg::OccupancyGrid & map) const
{
  if (is_occupied(map, start.x, start.y) ||
      is_occupied_with_clearance(map, goal.x, goal.y)) {
    return {};
  }
  const auto navigation_map = inflate_map(map);
  const double resolution = map.info.resolution;
  const double origin_x = map.info.origin.position.x;
  const double origin_y = map.info.origin.position.y;

  using Point = std::pair<int, int>; 
  struct Node;
  using NodePtr = std::shared_ptr<Node>;

  Point start_cell{
    static_cast<int>(std::floor((start.x - origin_x) / resolution)),
    static_cast<int>(std::floor((start.y - origin_y) / resolution))
  };

  Point goal_cell{
    static_cast<int>(std::floor((goal.x - origin_x) / resolution)),
    static_cast<int>(std::floor((goal.y - origin_y) / resolution))
  };

  struct Node
  {
    Point position;
    float cost;
    float score;
    NodePtr prev;

    Node(
      Point position,
      Point goal,
      float cost,
      NodePtr previous = nullptr)
      : position(position),
        cost(cost),
        score(cost + std::hypot(position.first - goal.first, position.second - goal.second)),
        prev(previous)
    {}

    struct Compare
    {
      bool operator()(
        const NodePtr& a,
        const NodePtr& b) const
      {
        return a->score > b->score;
      }
    };
  };

  std::priority_queue<NodePtr, std::vector<NodePtr>, Node::Compare> queue;

  const Point directions[8] = {
    {-1, -1}, {0, -1}, {1, -1},
    {-1,  0},          {1,  0},
    {-1,  1}, {0,  1}, {1,  1}
  };

  auto blocked = [&](Point cell) {
    return cell.first < 0 || cell.second < 0 ||
           cell.first >= static_cast<int>(map.info.width) ||
           cell.second >= static_cast<int>(map.info.height) ||
           navigation_map.data[cell.second * map.info.width + cell.first] != 0;
  };
  if (blocked(goal_cell)) {
    return {};
  }

  std::map<Point, float> best_cost;
  best_cost[start_cell] = 0.0f;
  queue.push(std::make_shared<Node>(start_cell, goal_cell, 0.0f));

  NodePtr reached = nullptr;
  while (!queue.empty()) {
    NodePtr previous = queue.top();
    queue.pop();

    if (previous->cost > best_cost.at(previous->position)) {
      continue;
    }
    if (previous->position == goal_cell) {
      reached = previous;
      break;
    }

    for (const Point& i : directions) {
      Point position{previous->position.first + i.first, previous->position.second + i.second};
      if (!is_navigation_step_valid(map, navigation_map,
          previous->position.first, previous->position.second, position.first, position.second)) {
        continue;
      }

      float cost = previous->cost + std::hypot(i.first, i.second);
      auto existing = best_cost.find(position);
      if (existing != best_cost.end() && cost >= existing->second) {
        continue;
      }
      best_cost[position] = cost;
      queue.push(std::make_shared<Node>(position, goal_cell, cost, previous));
    }
  }

  if (!reached) {
    return {};
  }

  Path path;
  for (NodePtr node = reached; node; node = node->prev) {
    geometry_msgs::msg::Point waypoint;
    waypoint.x = origin_x + (node->position.first + 0.5) * resolution;
    waypoint.y = origin_y + (node->position.second + 0.5) * resolution;
    waypoint.z = start.z;
    path.push_back(waypoint);
  }
  std::reverse(path.begin(), path.end());

  path.front() = start;
  if (path.size() == 1) {
    path.push_back(goal);
  } else {
    path.back() = goal;
  }

  // Distribute the climb/descent evenly over the route's waypoint intervals.
  const double height_step = (goal.z - start.z) / static_cast<double>(path.size() - 1);
  for (size_t i = 1; i + 1 < path.size(); ++i) {
    path[i].z = start.z + height_step * i;
  }

  return path;
}

}  // namespace ghost_brain
