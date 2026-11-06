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

#include <functional>
#include <optional>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace ghost_brain
{

/**
 * @brief Tours the house to find things when the ghost knows of nothing worth chasing.
 *
 * The ghost only finds people and candy by seeing them, and cannot see through walls. So it lays
 * a coarse grid of waypoints over the free space in /map, flies to the nearest one it has not
 * been to (nearest by how far it has to FLY, so doorways come out of the planner by themselves:
 * the rest of the room is always closer than the room next door), and looks around there.
 */
class Explorer
{
public:
  /// Forget the tour. Called at the start of every game
  void reset();

  /// Lay waypoints over the map's free space. Does nothing if they are already laid
  void build(const nav_msgs::msg::OccupancyGrid & map);

  bool built() const { return !waypoints_.empty(); }

  /// True once every waypoint has been visited or given up on
  bool done() const;

  /// Mark every waypoint within `radius` meters of `here` as visited
  void mark_visited(const geometry_msgs::msg::Point & here, double radius);

  /// Give up on a waypoint nobody can fly to
  void skip(const geometry_msgs::msg::Point & waypoint);

  /**
   * @brief The next waypoint to go to, or nothing if the tour is done.
   * @param cost how far the ghost has to FLY to a point (infinity if there is no route)
   */
  std::optional<geometry_msgs::msg::Point> next_waypoint(
    const std::function<double(const geometry_msgs::msg::Point &)> & cost) const;

private:
  struct Waypoint
  {
    geometry_msgs::msg::Point position;
    bool visited{false};
  };
  std::vector<Waypoint> waypoints_;
};

}  // namespace ghost_brain
