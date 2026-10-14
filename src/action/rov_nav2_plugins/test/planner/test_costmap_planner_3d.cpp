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
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rov_nav2_plugins/planner/costmap_planner_3d.hpp"
#include "test_fixtures/test_executor_fixture.hpp"

using Catch::Matchers::WithinAbs;
using rov_nav2_plugins::CostmapPlanner3D;
using wato::test::TestExecutorFixture;

namespace
{

geometry_msgs::msg::PoseStamped map_pose(double x, double y, double z)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;
  pose.pose.orientation.w = 1.0;
  return pose;
}

/// The planner and the costmap it plans on, configured as the planner server would. The costmap is
/// the default 5 x 5 m with its origin at (0, 0) in "map", and starts empty.
struct Setup
{
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap;
  std::unique_ptr<CostmapPlanner3D> planner;

  Setup()
  {
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("planner_test_node");
    costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_global_costmap");
    costmap->on_configure(rclcpp_lifecycle::State());

    planner = std::make_unique<CostmapPlanner3D>();
    planner->configure(node, "CostmapPlanner3D", nullptr, costmap);
    planner->activate();
  }

  /// Mark the cell under every point within `radius` of the line from (x0, y0) to (x1, y1) as lethal.
  void add_wall(double x0, double y0, double x1, double y1, double radius = 0.1)
  {
    auto * grid = costmap->getCostmap();
    for (unsigned int mx = 0; mx < grid->getSizeInCellsX(); ++mx) {
      for (unsigned int my = 0; my < grid->getSizeInCellsY(); ++my) {
        double wx, wy;
        grid->mapToWorld(mx, my, wx, wy);
        if (distance_to_segment(wx, wy, x0, y0, x1, y1) <= radius) {
          grid->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
        }
      }
    }
  }

  bool lethal_at(double wx, double wy) const
  {
    unsigned int mx, my;
    auto * grid = costmap->getCostmap();
    return grid->worldToMap(wx, wy, mx, my) && grid->getCost(mx, my) == nav2_costmap_2d::LETHAL_OBSTACLE;
  }

  static double distance_to_segment(double px, double py, double x0, double y0, double x1, double y1)
  {
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double length_sq = dx * dx + dy * dy;
    const double t = length_sq > 0.0 ? std::clamp(((px - x0) * dx + (py - y0) * dy) / length_sq, 0.0, 1.0) : 0.0;
    return std::hypot(px - (x0 + t * dx), py - (y0 + t * dy));
  }
};

bool never_canceled()
{
  return false;
}

}  // namespace

TEST_CASE_METHOD(TestExecutorFixture, "CostmapPlanner3D plans a 3D path through open water", "[planner]")
{
  Setup setup;

  const auto path = setup.planner->createPlan(map_pose(1, 1, -1), map_pose(4, 1, -2), never_canceled);

  REQUIRE(path.header.frame_id == "map");
  REQUIRE(path.poses.size() > 2);
  REQUIRE_THAT(path.poses.front().pose.position.z, WithinAbs(-1.0, 1e-9));
  REQUIRE_THAT(path.poses.back().pose.position.x, WithinAbs(4.0, 1e-9));
  REQUIRE_THAT(path.poses.back().pose.position.y, WithinAbs(1.0, 1e-9));
  REQUIRE_THAT(path.poses.back().pose.position.z, WithinAbs(-2.0, 1e-9));
  for (const auto & pose : path.poses) {
    REQUIRE(pose.header.frame_id == "map");
    // With nothing in the way, the route stays near the straight line
    REQUIRE_THAT(pose.pose.position.y, WithinAbs(1.0, 0.2));
  }
}

TEST_CASE_METHOD(TestExecutorFixture, "CostmapPlanner3D routes around an obstacle", "[planner]")
{
  Setup setup;
  // A wall across the straight line from (1, 1) to (4, 1), with a gap at the top
  setup.add_wall(2.5, 0.0, 2.5, 3.5);

  const auto path = setup.planner->createPlan(map_pose(1, 1, -1), map_pose(4, 1, -2), never_canceled);

  REQUIRE_THAT(path.poses.back().pose.position.x, WithinAbs(4.0, 1e-9));
  REQUIRE_THAT(path.poses.back().pose.position.z, WithinAbs(-2.0, 1e-9));
  double highest = 0.0;
  for (const auto & pose : path.poses) {
    REQUIRE_FALSE(setup.lethal_at(pose.pose.position.x, pose.pose.position.y));
    highest = std::max(highest, pose.pose.position.y);
  }
  // It went through the gap
  REQUIRE(highest > 3.5);
}

TEST_CASE_METHOD(TestExecutorFixture, "CostmapPlanner3D fails when obstacles block every route", "[planner]")
{
  Setup setup;
  // A box around the goal
  setup.add_wall(3.5, 0.5, 4.5, 0.5);
  setup.add_wall(4.5, 0.5, 4.5, 1.5);
  setup.add_wall(4.5, 1.5, 3.5, 1.5);
  setup.add_wall(3.5, 1.5, 3.5, 0.5);

  REQUIRE_THROWS_AS(
    setup.planner->createPlan(map_pose(1, 1, -1), map_pose(4, 1, -2), never_canceled), nav2_core::PlannerException);
}
