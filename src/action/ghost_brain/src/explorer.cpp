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


#include "ghost_brain/explorer.hpp"

#include <cmath>
#include <limits>

#include "ghost_brain/planner.hpp"

namespace ghost_brain
{

namespace
{

/// Spacing between waypoints (m), and how much free space (m) must surround one
constexpr double kWaypointSpacing = 2.5;
constexpr double kClearance = 0.8;  // wider than a 1.2 m doorway lets: no stops in doorways

/// The height the ghost looks around from (m)
constexpr double kLookHeight = 1.5;

/// Whether a disc of the given radius around (x, y) is free of anything in the map
bool is_clear(const nav_msgs::msg::OccupancyGrid & map, double x, double y, double radius)
{
  constexpr int kSamples = 8;
  if (is_occupied(map, x, y)) {
    return false;
  }
  for (int i = 0; i < kSamples; ++i) {
    const double angle = 2.0 * M_PI * i / kSamples;
    if (is_occupied(map, x + radius * std::cos(angle), y + radius * std::sin(angle))) {
      return false;
    }
  }
  return true;
}

}  // namespace

void Explorer::reset()
{
  waypoints_.clear();
}

void Explorer::build(const nav_msgs::msg::OccupancyGrid & map)
{
  if (built() || map.data.empty()) {
    return;
  }
  const double res = map.info.resolution;
  const double min_x = map.info.origin.position.x;
  const double min_y = map.info.origin.position.y;
  const double max_x = min_x + map.info.width * res;
  const double max_y = min_y + map.info.height * res;
  for (double y = min_y + kWaypointSpacing / 2.0; y < max_y; y += kWaypointSpacing) {
    for (double x = min_x + kWaypointSpacing / 2.0; x < max_x; x += kWaypointSpacing) {
      if (is_clear(map, x, y, kClearance)) {
        Waypoint waypoint;
        waypoint.position.x = x;
        waypoint.position.y = y;
        waypoint.position.z = kLookHeight;
        waypoints_.push_back(waypoint);
      }
    }
  }
}

bool Explorer::done() const
{
  for (const auto & waypoint : waypoints_) {
    if (!waypoint.visited) {
      return false;
    }
  }
  return true;
}

void Explorer::mark_visited(const geometry_msgs::msg::Point & here, double radius)
{
  for (auto & waypoint : waypoints_) {
    if (std::hypot(waypoint.position.x - here.x, waypoint.position.y - here.y) < radius) {
      waypoint.visited = true;
    }
  }
}

void Explorer::skip(const geometry_msgs::msg::Point & target)
{
  for (auto & waypoint : waypoints_) {
    if (std::hypot(waypoint.position.x - target.x, waypoint.position.y - target.y) < 0.01) {
      waypoint.visited = true;
    }
  }
}

std::optional<geometry_msgs::msg::Point> Explorer::next_waypoint(
  const std::function<double(const geometry_msgs::msg::Point &)> & cost) const
{
  // The cheapest unvisited waypoint to fly to. This is what clears a room before moving on: the
  // waypoints in the room the ghost is in are cheaper than the ones through the doorway
  std::optional<geometry_msgs::msg::Point> best;
  double best_cost = std::numeric_limits<double>::infinity();
  for (const auto & waypoint : waypoints_) {
    if (waypoint.visited) {
      continue;
    }
    const double c = cost(waypoint.position);
    if (c < best_cost) {  // infinity (no route) never beats the starting value
      best_cost = c;
      best = waypoint.position;
    }
  }
  return best;
}

}  // namespace ghost_brain
