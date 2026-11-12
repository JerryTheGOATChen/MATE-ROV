// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
#pragma once

#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>

namespace rov_nav2_plugins
{

struct OctomapMapperConfig
{
  double resolution{0.1};
  double max_range{8.0};
  double probability_hit{0.7};
  double probability_miss{0.4};
  double clamping_min{0.12};
  double clamping_max{0.97};
  double occupancy_threshold{0.5};
  bool compress_map{true};
};

/** @brief Owns and incrementally updates the planner's authoritative OctoMap. */
class OctomapMapper
{
public:
  explicit OctomapMapper(const OctomapMapperConfig & config);

  /**
   * Integrate endpoints expressed in the map frame.
   *
   * OctoMap performs the ray tracing, ensures occupied endpoints win over free rays within one
   * scan, and applies the configured hit/miss and clamping probabilities.
   */
  bool insertPointCloud(
    const octomap::point3d & sensor_origin,
    const octomap::Pointcloud & endpoints);

  octomap::OcTree & tree();
  const octomap::OcTree & tree() const;

private:
  OctomapMapperConfig config_;
  octomap::OcTree tree_;
};

}  // namespace rov_nav2_plugins
