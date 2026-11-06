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

// Tests for your planner and controller.
//
// Test cases tagged [.hackathon] are hidden until you are ready for them. Run them with
//   ./build/ghost_brain/test_planning "[hackathon]"
// and when your code passes, delete the "[hackathon]" tag's dot so they run with colcon test too.
// Add your own as you go: a test is the quickest way to try a planner without the whole house.

#include <cmath>
#include <utility>

#include <catch2/catch_test_macros.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "ghost_brain/controller.hpp"
#include "ghost_brain/planner.hpp"

using geometry_msgs::msg::Point;
using ghost_brain::Controller;
using ghost_brain::is_occupied;
using ghost_brain::Path;
using ghost_brain::Planner;
using haunted_house_msgs::msg::BoxArray;
using nav_msgs::msg::OccupancyGrid;

namespace
{

Point point(double x, double y, double z = 1.5)
{
  Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

/// A 10 x 10 m room at 0.1 m per cell, from (0, 0), with a wall across x = 5 from y = 0 to y = 8
OccupancyGrid room_with_wall()
{
  OccupancyGrid map;
  map.header.frame_id = "map";
  map.info.resolution = 0.1;
  map.info.width = 100;
  map.info.height = 100;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(100 * 100, 0);
  for (int row = 0; row < 80; ++row) {
    for (int col = 49; col <= 51; ++col) {
      map.data[row * 100 + col] = 100;
    }
  }
  return map;
}

/// The same wall as room_with_wall(), as a 3D box: x 4.9..5.2, y 0..8, floor to ceiling
BoxArray wall_boxes()
{
  haunted_house_msgs::msg::Box wall;
  wall.name = "wall";
  wall.min.x = 4.9;
  wall.min.y = 0.0;
  wall.min.z = 0.0;
  wall.max.x = 5.2;
  wall.max.y = 8.0;
  wall.max.z = 3.0;
  BoxArray boxes;
  boxes.boxes.push_back(wall);
  return boxes;
}

geometry_msgs::msg::Pose pose_at(double x, double y, double z, double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position = point(x, y, z);
  pose.orientation.z = std::sin(yaw / 2.0);
  pose.orientation.w = std::cos(yaw / 2.0);
  return pose;
}

double distance(const Point & a, const Point & b)
{
  return std::hypot(a.x - b.x, a.y - b.y, a.z - b.z);
}

}  // namespace

TEST_CASE("is_occupied reads the map", "[planning]")
{
  const OccupancyGrid map = room_with_wall();
  REQUIRE(is_occupied(map, 5.0, 4.0));
  REQUIRE_FALSE(is_occupied(map, 2.0, 4.0));
  REQUIRE_FALSE(is_occupied(map, 5.0, 9.0));
  // Off the edge of the map counts as blocked
  REQUIRE(is_occupied(map, -1.0, 4.0));
  REQUIRE(is_occupied(map, 4.0, 10.5));
}

TEST_CASE("The controller stops when there is nowhere to go", "[planning]")
{
  Controller controller;
  const auto twist = controller.compute_velocity(pose_at(1.0, 1.0, 1.5, 0.0), Path{});
  REQUIRE(twist.linear.x == 0.0);
  REQUIRE(twist.linear.y == 0.0);
  REQUIRE(twist.linear.z == 0.0);
  REQUIRE(twist.angular.z == 0.0);
}

TEST_CASE("The planner finds a route around a wall", "[hackathon]")
{
  const Planner planner;
  const OccupancyGrid map = room_with_wall();
  const Point start = point(2.0, 2.0);
  const Point goal = point(8.0, 2.0);

  const Path path = planner.plan(start, goal, map, wall_boxes());

  REQUIRE_FALSE(path.empty());
  REQUIRE(distance(path.front(), start) < 0.5);
  REQUIRE(distance(path.back(), goal) < 0.5);
  for (const auto & p : path) {
    REQUIRE_FALSE(is_occupied(map, p.x, p.y));
  }
  // It has to go around the end of the wall, at y = 8
  bool went_around = false;
  for (const auto & p : path) {
    went_around = went_around || p.y > 8.0;
  }
  REQUIRE(went_around);
}

TEST_CASE("The planner moves a goal inside a wall to open space next to it", "[hackathon]")
{
  const Planner planner;
  const Point goal = point(5.0, 4.0);
  const Path path = planner.plan(point(2.0, 2.0), goal, room_with_wall(), wall_boxes());
  // A drifted estimate can put a target in a wall: get as close as possible instead of freezing
  REQUIRE_FALSE(path.empty());
  REQUIRE(distance(path.back(), goal) < 0.7);
  REQUIRE_FALSE(is_occupied(room_with_wall(), path.back().x, path.back().y));
}

TEST_CASE("The planner can start from inside a wall's safety margin", "[hackathon]")
{
  const Planner planner;
  // 0.15 m from the wall's face: closer than the ghost's radius, as drift or a bonk can leave it
  const Point start = point(4.75, 2.0);
  const Path path = planner.plan(start, point(2.0, 2.0), room_with_wall(), wall_boxes());
  REQUIRE_FALSE(path.empty());
  REQUIRE(distance(path.back(), point(2.0, 2.0)) < 0.5);
}

TEST_CASE("The planner flies through a 1.2 m doorway", "[hackathon]")
{
  const Planner planner;
  // A solid wall across x = 5 from y = 0 to 10, with a 1.2 m door at y 4.4 to 5.6
  OccupancyGrid map = room_with_wall();
  for (auto & cell : map.data) {
    cell = 0;
  }
  BoxArray boxes;
  for (const auto & span : {std::pair<double, double>{0.0, 4.4}, std::pair<double, double>{5.6, 10.0}}) {
    haunted_house_msgs::msg::Box part;
    part.name = "wall";
    part.min.x = 4.9;
    part.max.x = 5.1;
    part.min.y = span.first;
    part.max.y = span.second;
    part.min.z = 0.0;
    part.max.z = 3.0;
    boxes.boxes.push_back(part);
    for (int row = static_cast<int>(span.first * 10); row < static_cast<int>(span.second * 10); ++row) {
      for (int col = 49; col <= 51; ++col) {
        map.data[row * 100 + col] = 100;
      }
    }
  }
  const Path path = planner.plan(point(2.0, 5.0), point(8.0, 5.0), map, boxes);
  REQUIRE_FALSE(path.empty());
  REQUIRE(distance(path.back(), point(8.0, 5.0)) < 0.5);
}

TEST_CASE("The planner heads for the nearest reachable point when the goal is walled off", "[hackathon]")
{
  const Planner planner;
  // A box room with no door: the goal inside it cannot be reached
  OccupancyGrid map = room_with_wall();
  for (auto & cell : map.data) {
    cell = 0;
  }
  BoxArray boxes;
  const double spans[4][4] = {{6.0, 6.0, 9.0, 6.2}, {6.0, 9.0, 9.0, 9.2}, {6.0, 6.0, 6.2, 9.2}, {9.0, 6.0, 9.2, 9.2}};
  for (const auto & b : spans) {
    haunted_house_msgs::msg::Box part;
    part.name = "wall";
    part.min.x = b[0];
    part.min.y = b[1];
    part.max.x = b[2];
    part.max.y = b[3];
    part.min.z = 0.0;
    part.max.z = 3.0;
    boxes.boxes.push_back(part);
  }
  const Path path = planner.plan(point(2.0, 2.0), point(7.5, 7.5), map, boxes);
  // Not empty: it goes as close as it can get, which is outside the room
  REQUIRE_FALSE(path.empty());
  REQUIRE(distance(path.back(), point(7.5, 7.5)) < distance(point(2.0, 2.0), point(7.5, 7.5)));
}

TEST_CASE("The controller flies toward the route", "[hackathon]")
{
  Controller controller;
  const Path path = {point(1.0, 1.0, 1.5), point(4.0, 1.0, 2.0)};

  SECTION("Straight ahead and up: it goes forward and climbs")
  {
    const auto twist = controller.compute_velocity(pose_at(1.0, 1.0, 1.5, 0.0), path);
    REQUIRE(twist.linear.x > 0.0);
    REQUIRE(twist.linear.z > 0.0);
  }

  SECTION("Facing the wrong way: it turns, or flies backwards")
  {
    const auto twist = controller.compute_velocity(pose_at(1.0, 1.0, 1.5, M_PI), path);
    REQUIRE((twist.angular.z != 0.0 || twist.linear.x < 0.0));
  }
}
