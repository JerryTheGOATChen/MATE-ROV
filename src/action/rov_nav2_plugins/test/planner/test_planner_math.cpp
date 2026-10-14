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
#include <cmath>
#include <cstddef>
#include <Eigen/Geometry>
#include <stdexcept>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "angles/angles.h"
#include "rov_nav2_plugins/planner/planner_math.hpp"

using Catch::Matchers::WithinAbs;
using rov_nav2_plugins::add_depth;
using rov_nav2_plugins::Pose4D;

namespace
{

constexpr double kTolerance = 1e-9;

void require_same_pose(const Pose4D & actual, const Pose4D & expected)
{
  REQUIRE_THAT((actual.position - expected.position).norm(), WithinAbs(0.0, kTolerance));
  REQUIRE_THAT(angles::shortest_angular_distance(actual.yaw, expected.yaw), WithinAbs(0.0, kTolerance));
}

/// An L-shaped detour from (0, 0) to (2, 2) through (2, 0), as a 2D planner might return it: the ends
/// are cell centers, slightly off the true start and goal.
std::vector<Eigen::Vector2d> l_shaped_path()
{
  return {{0.05, 0.05}, {1.0, 0.0}, {2.0, 0.0}, {2.0, 1.0}, {1.95, 1.95}};
}

}  // namespace

TEST_CASE("add_depth starts at the start and ends exactly at the goal", "[planner_math]")
{
  const Pose4D start{{0.0, 0.0, -1.0}, 0.0};
  const Pose4D goal{{2.0, 2.0, -3.0}, M_PI_2};

  for (const bool z_before_xy : {false, true}) {
    const auto path = add_depth(l_shaped_path(), start, goal, z_before_xy);
    require_same_pose(path.front(), start);
    require_same_pose(path.back(), goal);
  }
}

TEST_CASE("add_depth keeps the 2D route and changes depth with horizontal distance", "[planner_math]")
{
  const Pose4D start{{0.0, 0.0, -1.0}, 0.0};
  const Pose4D goal{{2.0, 2.0, -3.0}, 0.0};

  const auto path = add_depth(l_shaped_path(), start, goal, false);

  // One pose per 2D point
  REQUIRE(path.size() == 5);
  REQUIRE_THAT(path[2].position.x(), WithinAbs(2.0, kTolerance));
  REQUIRE_THAT(path[2].position.y(), WithinAbs(0.0, kTolerance));
  // Halfway along the 4 m route, depth is halfway between -1 and -3
  REQUIRE_THAT(path[2].position.z(), WithinAbs(-2.0, kTolerance));
  for (std::size_t i = 1; i < path.size(); ++i) {
    REQUIRE(path[i].position.z() < path[i - 1].position.z());
  }
}

TEST_CASE("add_depth can change depth before moving horizontally", "[planner_math]")
{
  const Pose4D start{{0.0, 0.0, -1.0}, 0.0};
  const Pose4D goal{{2.0, 2.0, -3.0}, 0.0};

  const auto path = add_depth(l_shaped_path(), start, goal, true);

  // Straight down at the start, then the 4 remaining 2D points at the goal's depth
  REQUIRE(path.size() == 6);
  REQUIRE_THAT(path[1].position.head<2>().norm(), WithinAbs(0.0, kTolerance));
  for (std::size_t i = 1; i < path.size(); ++i) {
    REQUIRE_THAT(path[i].position.z(), WithinAbs(-3.0, kTolerance));
  }
}

TEST_CASE("add_depth turns toward the goal's yaw along the path", "[planner_math]")
{
  const Pose4D start{{0.0, 0.0, 0.0}, 0.0};
  const Pose4D goal{{2.0, 2.0, 0.0}, M_PI_2};

  const auto path = add_depth(l_shaped_path(), start, goal, false);

  // Halfway along the route, halfway through the turn
  REQUIRE_THAT(path[2].yaw, WithinAbs(M_PI_4, kTolerance));
  for (std::size_t i = 1; i < path.size(); ++i) {
    REQUIRE(path[i].yaw > path[i - 1].yaw);
  }
}

TEST_CASE("add_depth turns the short way", "[planner_math]")
{
  // From 3.0 rad to -3.0 rad, the short way passes through pi rather than 0
  const std::vector<Eigen::Vector2d> xy_path{{0.0, 0.0}, {0.5, 0.0}, {1.0, 0.0}};
  const auto path = add_depth(xy_path, {{0.0, 0.0, 0.0}, 3.0}, {{1.0, 0.0, 0.0}, -3.0}, false);

  REQUIRE(path.size() == 3);
  REQUIRE_THAT(std::abs(path[1].yaw), WithinAbs(M_PI, 1e-6));
}

TEST_CASE("add_depth changes depth in place when the route has no horizontal move", "[planner_math]")
{
  const Pose4D start{{1.0, 1.0, 0.0}, 0.0};
  const Pose4D goal{{1.0, 1.0, -2.0}, 1.0};

  // A 2D planner returns a single point when start and goal share a cell
  const auto path = add_depth({{1.0, 1.0}}, start, goal, false);

  require_same_pose(path.front(), start);
  require_same_pose(path.back(), goal);
  for (std::size_t i = 1; i < path.size(); ++i) {
    REQUIRE_THAT((path[i].position - start.position).head<2>().norm(), WithinAbs(0.0, kTolerance));
    REQUIRE(path[i].position.z() <= path[i - 1].position.z());
  }
}

TEST_CASE("add_depth turns in place when start and goal coincide", "[planner_math]")
{
  const Pose4D start{{1.0, 1.0, -1.0}, 0.0};
  const Pose4D goal{{1.0, 1.0, -1.0}, 2.0};

  const auto path = add_depth({{1.0, 1.0}}, start, goal, false);

  require_same_pose(path.front(), start);
  require_same_pose(path.back(), goal);
  for (const Pose4D & pose : path) {
    REQUIRE_THAT((pose.position - start.position).norm(), WithinAbs(0.0, kTolerance));
  }
}

TEST_CASE("add_depth rejects an empty 2D path", "[planner_math]")
{
  REQUIRE_THROWS_AS(add_depth({}, Pose4D{}, Pose4D{}, false), std::invalid_argument);
}
