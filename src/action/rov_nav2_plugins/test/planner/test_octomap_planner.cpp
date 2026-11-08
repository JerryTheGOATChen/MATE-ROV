// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
#include <algorithm>
#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <octomap/OcTree.h>

#include "rov_nav2_plugins/planner/octomap_planner.hpp"

using rov_nav2_plugins::OctomapPlanner;
using rov_nav2_plugins::OctomapPlannerConfig;

namespace
{

OctomapPlannerConfig config()
{
  OctomapPlannerConfig value;
  value.planning_resolution = 0.25;
  value.collision_radius = 0.18;
  value.min_bounds = Eigen::Vector3d(0.0, 0.0, -4.0);
  value.max_bounds = Eigen::Vector3d(5.0, 5.0, 0.0);
  value.max_expansions = 100000;
  return value;
}

void add_wall(
  octomap::OcTree & tree,
  double min_z,
  double max_z)
{
  for (double x = 2.4; x <= 2.6; x += 0.1) {
    for (double y = 0.2; y <= 4.8; y += 0.1) {
      for (double z = min_z; z <= max_z; z += 0.1) {
        tree.updateNode(octomap::point3d(x, y, z), true);
      }
    }
  }
  tree.updateInnerOccupancy();
}

}  // namespace

TEST_CASE("OctoMap planner returns a direct path through open water", "[octomap_planner]")
{
  octomap::OcTree tree(0.1);
  OctomapPlanner planner(config());
  const Eigen::Vector3d start(0.5, 2.5, -2.0);
  const Eigen::Vector3d goal(4.5, 2.5, -2.0);

  const auto path = planner.plan(tree, start, goal);

  REQUIRE(path.size() == 2U);
  REQUIRE(path.front().isApprox(start));
  REQUIRE(path.back().isApprox(goal));
}

TEST_CASE("OctoMap planner goes above or below a suspended obstacle", "[octomap_planner]")
{
  octomap::OcTree tree(0.1);
  add_wall(tree, -2.35, -1.65);
  OctomapPlanner planner(config());

  const auto path = planner.plan(
    tree,
    Eigen::Vector3d(0.5, 2.5, -2.0),
    Eigen::Vector3d(4.5, 2.5, -2.0));

  REQUIRE(path.size() >= 3U);
  REQUIRE(std::any_of(path.begin(), path.end(), [](const Eigen::Vector3d & point) {
      return point.z() < -2.55 || point.z() > -1.45;
    }));
  for (const auto & point : path) {
    REQUIRE(planner.isStateFree(tree, point));
  }
}

TEST_CASE("OctoMap planner fails when a wall spans the water column", "[octomap_planner]")
{
  octomap::OcTree tree(0.1);
  add_wall(tree, -3.8, -0.2);
  OctomapPlanner planner(config());

  const auto path = planner.plan(
    tree,
    Eigen::Vector3d(0.5, 2.5, -2.0),
    Eigen::Vector3d(4.5, 2.5, -2.0));

  REQUIRE(path.empty());
}
