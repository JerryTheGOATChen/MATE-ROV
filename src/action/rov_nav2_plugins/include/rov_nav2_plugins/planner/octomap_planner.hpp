// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <mutex>
#include <vector>

#include <Eigen/Core>
#include <octomap/OcTree.h>

namespace rov_nav2_plugins
{

struct OctomapPlannerConfig
{
  double planning_resolution{0.25};
  double collision_radius{0.45};
  Eigen::Vector3d min_bounds{-11.0, -7.0, -4.0};
  Eigen::Vector3d max_bounds{11.0, 7.0, 0.0};
  bool unknown_is_occupied{false};
  std::size_t max_expansions{250000};
};

/**
 * @brief A 26-connected A* search through an OctoMap.
 *
 * The vehicle is approximated as a sphere. Every candidate state and simplified path segment is
 * checked against occupied OctoMap cells in full x, y and z, so routes may pass over or under an
 * obstacle instead of flattening it into a 2D costmap.
 */
class OctomapPlanner
{
public:
  explicit OctomapPlanner(OctomapPlannerConfig config);

  std::vector<Eigen::Vector3d> plan(
    const octomap::OcTree & tree,
    std::uint64_t map_revision,
    const Eigen::Vector3d & start,
    const Eigen::Vector3d & goal,
    const std::function<bool()> & cancel_checker = {});

  bool isStateFree(const octomap::OcTree & tree, const Eigen::Vector3d & point) const;

  /** @brief Number of collision-lattice rebuilds, exposed for diagnostics and tests. */
  std::size_t collisionCacheBuildCount() const;

private:
  OctomapPlannerConfig config_;
  mutable std::mutex cache_mutex_;
  std::uint64_t cached_map_revision_{std::numeric_limits<std::uint64_t>::max()};
  std::vector<std::uint8_t> blocked_cache_;
  std::size_t collision_cache_build_count_{0};
};

}  // namespace rov_nav2_plugins
