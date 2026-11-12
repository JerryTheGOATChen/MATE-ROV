// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
#include <catch2/catch_test_macros.hpp>
#include <octomap/Pointcloud.h>

#include "rov_nav2_plugins/planner/octomap_mapper.hpp"

using rov_nav2_plugins::OctomapMapper;
using rov_nav2_plugins::OctomapMapperConfig;

TEST_CASE("OctoMap mapper marks ray cells free and endpoints occupied", "[octomap_mapper]")
{
  OctomapMapperConfig config;
  config.resolution = 0.1;
  config.compress_map = false;
  OctomapMapper mapper(config);
  octomap::Pointcloud endpoints;
  endpoints.push_back(1.0F, 0.0F, 0.0F);

  REQUIRE(mapper.insertPointCloud(octomap::point3d(0.0F, 0.0F, 0.0F), endpoints));

  const auto * free_node = mapper.tree().search(0.5, 0.0, 0.0);
  const auto * occupied_node = mapper.tree().search(1.0, 0.0, 0.0);
  REQUIRE(free_node != nullptr);
  REQUIRE_FALSE(mapper.tree().isNodeOccupied(*free_node));
  REQUIRE(occupied_node != nullptr);
  REQUIRE(mapper.tree().isNodeOccupied(*occupied_node));
}

TEST_CASE("OctoMap mapper treats returns beyond maximum range as free only", "[octomap_mapper]")
{
  OctomapMapperConfig config;
  config.resolution = 0.1;
  config.max_range = 0.8;
  config.compress_map = false;
  OctomapMapper mapper(config);
  octomap::Pointcloud endpoints;
  endpoints.push_back(2.0F, 0.0F, 0.0F);

  REQUIRE(mapper.insertPointCloud(octomap::point3d(0.0F, 0.0F, 0.0F), endpoints));

  const auto * range_node = mapper.tree().search(0.75, 0.0, 0.0);
  REQUIRE(range_node != nullptr);
  REQUIRE_FALSE(mapper.tree().isNodeOccupied(*range_node));
  REQUIRE(mapper.tree().search(2.0, 0.0, 0.0) == nullptr);
}

TEST_CASE("OctoMap mapper applies repeated hit and miss probabilities", "[octomap_mapper]")
{
  OctomapMapperConfig config;
  config.resolution = 0.1;
  config.max_range = 2.0;
  config.compress_map = false;
  OctomapMapper mapper(config);
  octomap::Pointcloud near_endpoint;
  near_endpoint.push_back(0.5F, 0.0F, 0.0F);
  octomap::Pointcloud far_endpoint;
  far_endpoint.push_back(1.0F, 0.0F, 0.0F);
  const octomap::point3d origin(0.0F, 0.0F, 0.0F);

  REQUIRE(mapper.insertPointCloud(origin, near_endpoint));
  const auto * node_after_hit = mapper.tree().search(0.5, 0.0, 0.0);
  REQUIRE(node_after_hit != nullptr);
  REQUIRE(mapper.tree().isNodeOccupied(*node_after_hit));

  REQUIRE(mapper.insertPointCloud(origin, far_endpoint));
  REQUIRE(mapper.insertPointCloud(origin, far_endpoint));
  REQUIRE(mapper.insertPointCloud(origin, far_endpoint));
  const auto * node_after_misses = mapper.tree().search(0.5, 0.0, 0.0);
  REQUIRE(node_after_misses != nullptr);
  REQUIRE_FALSE(mapper.tree().isNodeOccupied(*node_after_misses));
}
