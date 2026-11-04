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

#include <cmath>

namespace ghost_brain
{

namespace {

struct Node
{
  int col = 0;
  int row = 0;
  int index = 0;     // 1D index: row * width + col
  double g_cost = 0.0;
  double f_cost = 0.0; // g_cost + h_cost

  // For std::priority_queue (min-heap: smallest f_cost popped first)
  bool operator>(const Node & other) const
  {
    return f_cost > other.f_cost;
  }
};

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
  // TODO(hackathon): find a route from start to goal around the walls
  (void)start;
  (void)goal;
  (void)map;
  return {};
}

}  // namespace ghost_brain
