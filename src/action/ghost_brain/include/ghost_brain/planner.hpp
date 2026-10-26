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

#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace ghost_brain
{

/// A route for the ghost: points in the map frame, from where it is to where it is going.
using Path = std::vector<geometry_msgs::msg::Point>;

/**
 * @brief Whether the map marks a point as blocked by a wall or furniture.
 *
 * The map is the house seen from above, so furniture you could fly over counts as blocked too.
 * Points outside the map count as blocked.
 */
bool is_occupied(const nav_msgs::msg::OccupancyGrid & map, double x, double y);

/**
 * @brief Plans the ghost's route through the house. This is yours to write!
 *
 * The map is a nav_msgs/OccupancyGrid: cells of `map.info.resolution` meters, laid out row by row
 * from `map.info.origin`, each 100 where something stands and 0 where it is clear. The ghost is a
 * sphere 0.3 m in radius, so a route should keep that far from anything blocked.
 *
 * Some ideas, from simplest to fanciest:
 *  - A straight line from start to goal. Works until there is a wall in the way
 *  - Breadth-first search, Dijkstra or A* on the grid
 *  - Smooth the result, so the controller does not have to zig-zag
 *  - Go 3D: /haunted_house/obstacles lists every wall and piece of furniture as a 3D box, and
 *    a lot of furniture can be flown over or under
 */
class Planner
{
public:
  /// @return the route from start to goal, or an empty path if there is none
  Path plan(
    const geometry_msgs::msg::Point & start,
    const geometry_msgs::msg::Point & goal,
    const nav_msgs::msg::OccupancyGrid & map) const;
};

}  // namespace ghost_brain
