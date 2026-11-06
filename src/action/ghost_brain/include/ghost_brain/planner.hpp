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

#include <map>
#include <optional>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "haunted_house_msgs/msg/box_array.hpp"

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

  Path plan(
    const geometry_msgs::msg::Point & start,
    const geometry_msgs::msg::Point & goal,
    const nav_msgs::msg::OccupancyGrid & map,
    const haunted_house_msgs::msg::BoxArray & obstacles) const;
};

/// Left-wall exploration with a coverage-based escape from closed circuits and stalls.
class Explorer
{
public:
  geometry_msgs::msg::Twist update(
    const geometry_msgs::msg::Pose & pose,
    const nav_msgs::msg::OccupancyGrid & map, double time);
  void reset();
  /// Cancel exploration navigation when pursuing a target, retaining coverage memory.
  void interrupt();
  const Path & path() const { return path_; }
  bool escaping() const { return escape_goal_.has_value(); }

private:
  struct Visit
  {
    geometry_msgs::msg::Point position;
    double yaw;
    double time;
    double travel;
  };
  bool start_escape(
    const geometry_msgs::msg::Point & here,
    const nav_msgs::msg::OccupancyGrid & map, double time);
  std::map<std::pair<int, int>, unsigned> coverage_;
  std::vector<Visit> history_;
  std::optional<Visit> progress_;
  std::optional<geometry_msgs::msg::Point> escape_goal_;
  double escape_started_{0.0};
  double last_plan_{0.0};
  double travel_{0.0};
  bool following_wall_{false};
  Path path_;
  Planner planner_;
};

}  // namespace ghost_brain
