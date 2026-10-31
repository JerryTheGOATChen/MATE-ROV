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
  // TODO(hackathon): find a route from start to goal around the walls

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
           map.data[cell.second * map.info.width + cell.first] != 0;
  };
  if (blocked(start_cell) || blocked(goal_cell)) {
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
      if (blocked(position)) {
        continue;
      }
      // Prevent diagonal moves from cutting through blocked corners.
      if (i.first != 0 && i.second != 0 &&
          (blocked({position.first, previous->position.second}) ||
           blocked({previous->position.first, position.second}))) {
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
  geometry_msgs::msg::Point horizontal_goal = goal;
  horizontal_goal.z = start.z;
  if (path.size() == 1) {
    path.push_back(horizontal_goal);
  } else {
    path.back() = horizontal_goal;
  }
  if (goal.z != start.z) {
    path.push_back(goal);
  }

  return path;
}

}  // namespace ghost_brain
