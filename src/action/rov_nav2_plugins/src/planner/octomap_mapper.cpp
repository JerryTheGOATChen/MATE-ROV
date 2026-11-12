// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
#include "rov_nav2_plugins/planner/octomap_mapper.hpp"

#include <stdexcept>

namespace rov_nav2_plugins
{

OctomapMapper::OctomapMapper(const OctomapMapperConfig & config)
: config_(config), tree_(config.resolution)
{
  if (config_.resolution <= 0.0) {
    throw std::invalid_argument("OctoMap resolution must be positive");
  }
  if (config_.max_range == 0.0) {
    throw std::invalid_argument("OctoMap maximum range must be positive or negative for unlimited");
  }
  if (config_.probability_hit <= 0.5 || config_.probability_hit > 1.0) {
    throw std::invalid_argument("OctoMap hit probability must be in (0.5, 1.0]");
  }
  if (config_.probability_miss < 0.0 || config_.probability_miss >= 0.5) {
    throw std::invalid_argument("OctoMap miss probability must be in [0.0, 0.5)");
  }
  if (config_.clamping_min < 0.0 || config_.clamping_max > 1.0 ||
    config_.clamping_min >= config_.clamping_max)
  {
    throw std::invalid_argument("OctoMap clamping probabilities are invalid");
  }
  if (config_.occupancy_threshold <= 0.0 || config_.occupancy_threshold >= 1.0) {
    throw std::invalid_argument("OctoMap occupancy threshold must be in (0.0, 1.0)");
  }

  tree_.setProbHit(config_.probability_hit);
  tree_.setProbMiss(config_.probability_miss);
  tree_.setClampingThresMin(config_.clamping_min);
  tree_.setClampingThresMax(config_.clamping_max);
  tree_.setOccupancyThres(config_.occupancy_threshold);
}

bool OctomapMapper::insertPointCloud(
  const octomap::point3d & sensor_origin,
  const octomap::Pointcloud & endpoints)
{
  if (endpoints.size() == 0U) {
    return false;
  }

  // This is the OctoMap library's batch insertion path. It computes free rays and occupied
  // endpoints once per key, gives occupied endpoints priority, and updates inner occupancy.
  tree_.insertPointCloud(endpoints, sensor_origin, config_.max_range, false, false);
  if (config_.compress_map) {
    tree_.prune();
  }
  return true;
}

octomap::OcTree & OctomapMapper::tree()
{
  return tree_;
}

const octomap::OcTree & OctomapMapper::tree() const
{
  return tree_;
}

}  // namespace rov_nav2_plugins
