// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
#include "rov_nav2_plugins/planner/octomap_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>

namespace rov_nav2_plugins
{
namespace
{

struct GridIndex
{
  int x;
  int y;
  int z;

  bool operator==(const GridIndex & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct QueueEntry
{
  float priority;
  std::uint32_t cell;
};

struct GreaterPriority
{
  bool operator()(const QueueEntry & lhs, const QueueEntry & rhs) const
  {
    return lhs.priority > rhs.priority;
  }
};

}  // namespace

OctomapPlanner::OctomapPlanner(OctomapPlannerConfig config)
: config_(std::move(config))
{
  if (config_.planning_resolution <= 0.0) {
    throw std::invalid_argument("planning_resolution must be positive");
  }
  if (config_.collision_radius < 0.0) {
    throw std::invalid_argument("collision_radius cannot be negative");
  }
  if (config_.max_expansions == 0U) {
    throw std::invalid_argument("max_expansions must be positive");
  }
  for (int axis = 0; axis < 3; ++axis) {
    if (config_.min_bounds[axis] >= config_.max_bounds[axis]) {
      throw std::invalid_argument("each minimum planning bound must be below its maximum");
    }
  }
}

bool OctomapPlanner::isStateFree(
  const octomap::OcTree & tree,
  const Eigen::Vector3d & point) const
{
  // Keep the complete spherical vehicle inside the configured water volume.
  for (int axis = 0; axis < 3; ++axis) {
    if (point[axis] - config_.collision_radius < config_.min_bounds[axis] ||
      point[axis] + config_.collision_radius > config_.max_bounds[axis])
    {
      return false;
    }
  }

  const double sample_resolution = tree.getResolution();
  const int sample_radius =
    std::max(0, static_cast<int>(std::ceil(config_.collision_radius / sample_resolution)));
  const double occupied_test_radius =
    config_.collision_radius + std::sqrt(3.0) * sample_resolution * 0.5;
  const double radius_squared = occupied_test_radius * occupied_test_radius;

  for (int ix = -sample_radius - 1; ix <= sample_radius + 1; ++ix) {
    for (int iy = -sample_radius - 1; iy <= sample_radius + 1; ++iy) {
      for (int iz = -sample_radius - 1; iz <= sample_radius + 1; ++iz) {
        const Eigen::Vector3d offset(
          ix * sample_resolution,
          iy * sample_resolution,
          iz * sample_resolution);
        if (offset.squaredNorm() > radius_squared) {
          continue;
        }

        const Eigen::Vector3d query = point + offset;
        const auto * node = tree.search(query.x(), query.y(), query.z());
        if (node == nullptr) {
          if (config_.unknown_is_occupied) {
            return false;
          }
        } else if (tree.isNodeOccupied(*node)) {
          return false;
        }
      }
    }
  }
  return true;
}

std::vector<Eigen::Vector3d> OctomapPlanner::plan(
  const octomap::OcTree & tree,
  const Eigen::Vector3d & start,
  const Eigen::Vector3d & goal,
  const std::function<bool()> & cancel_checker) const
{
  if (!isStateFree(tree, start) || !isStateFree(tree, goal)) {
    return {};
  }
  if ((goal - start).norm() < 1e-9) {
    return {start};
  }

  const auto to_index = [this](const Eigen::Vector3d & point) {
      return GridIndex{
        static_cast<int>(std::lround((point.x() - config_.min_bounds.x()) /
        config_.planning_resolution)),
        static_cast<int>(std::lround((point.y() - config_.min_bounds.y()) /
        config_.planning_resolution)),
        static_cast<int>(std::lround((point.z() - config_.min_bounds.z()) /
        config_.planning_resolution))};
    };
  const auto to_point = [this](const GridIndex & index) -> Eigen::Vector3d {
      return config_.min_bounds + config_.planning_resolution * Eigen::Vector3d(
        static_cast<double>(index.x),
        static_cast<double>(index.y),
        static_cast<double>(index.z));
    };

  // The search volume is small at planning resolution (about 86k cells for the default pool).
  // Compact arrays are substantially faster and smaller than three unordered containers holding
  // GridIndex keys. The OctoMap remains the authoritative, sparse obstacle representation; this
  // lattice is only short-lived A* workspace.
  const GridIndex dimensions{
    static_cast<int>(std::floor(
      (config_.max_bounds.x() - config_.min_bounds.x()) / config_.planning_resolution)) + 1,
    static_cast<int>(std::floor(
      (config_.max_bounds.y() - config_.min_bounds.y()) / config_.planning_resolution)) + 1,
    static_cast<int>(std::floor(
      (config_.max_bounds.z() - config_.min_bounds.z()) / config_.planning_resolution)) + 1};
  const std::size_t cell_count =
    static_cast<std::size_t>(dimensions.x) * static_cast<std::size_t>(dimensions.y) *
    static_cast<std::size_t>(dimensions.z);
  if (cell_count > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())) {
    throw std::runtime_error("OctomapPlanner planning lattice is too large");
  }

  const auto index_in_bounds = [&dimensions](const GridIndex & index) {
      return index.x >= 0 && index.x < dimensions.x &&
             index.y >= 0 && index.y < dimensions.y &&
             index.z >= 0 && index.z < dimensions.z;
    };
  const auto flatten = [&dimensions](const GridIndex & index) {
      return static_cast<std::uint32_t>(
        (static_cast<std::size_t>(index.z) * static_cast<std::size_t>(dimensions.y) +
        static_cast<std::size_t>(index.y)) * static_cast<std::size_t>(dimensions.x) +
        static_cast<std::size_t>(index.x));
    };
  const auto expand = [&dimensions](std::uint32_t cell) {
      const int x = static_cast<int>(cell % static_cast<std::uint32_t>(dimensions.x));
      const std::uint32_t yz = cell / static_cast<std::uint32_t>(dimensions.x);
      const int y = static_cast<int>(yz % static_cast<std::uint32_t>(dimensions.y));
      const int z = static_cast<int>(yz / static_cast<std::uint32_t>(dimensions.y));
      return GridIndex{x, y, z};
    };

  const GridIndex start_index = to_index(start);
  const GridIndex goal_index = to_index(goal);
  if (!index_in_bounds(start_index) || !index_in_bounds(goal_index)) {
    return {};
  }
  if (start_index == goal_index) {
    return {};
  }

  // Inflate occupied OctoMap leaves once into one byte per planning cell. This is a compact
  // collision cache, not a second persistent voxel map. Half a cell diagonal conservatively
  // protects continuous segments that pass between lattice samples.
  std::vector<std::uint8_t> blocked(cell_count, 0U);
  const double edge_margin = std::sqrt(3.0) * config_.planning_resolution * 0.5;
  for (int z = 0; z < dimensions.z; ++z) {
    for (int y = 0; y < dimensions.y; ++y) {
      for (int x = 0; x < dimensions.x; ++x) {
        const GridIndex index{x, y, z};
        const Eigen::Vector3d point = to_point(index);
        for (int axis = 0; axis < 3; ++axis) {
          if (point[axis] - config_.collision_radius < config_.min_bounds[axis] ||
            point[axis] + config_.collision_radius > config_.max_bounds[axis])
          {
            blocked[flatten(index)] = 1U;
            break;
          }
        }
      }
    }
  }

  for (auto leaf = tree.begin_leafs(), end = tree.end_leafs(); leaf != end; ++leaf) {
    if (!tree.isNodeOccupied(*leaf)) {
      continue;
    }
    const Eigen::Vector3d center(leaf.getX(), leaf.getY(), leaf.getZ());
    const double leaf_half_size = leaf.getSize() * 0.5;
    const double inflation = config_.collision_radius + edge_margin + leaf_half_size;
    GridIndex min_index = to_index(center - Eigen::Vector3d::Constant(inflation));
    GridIndex max_index = to_index(center + Eigen::Vector3d::Constant(inflation));
    min_index.x = std::max(0, min_index.x);
    min_index.y = std::max(0, min_index.y);
    min_index.z = std::max(0, min_index.z);
    max_index.x = std::min(dimensions.x - 1, max_index.x);
    max_index.y = std::min(dimensions.y - 1, max_index.y);
    max_index.z = std::min(dimensions.z - 1, max_index.z);
    for (int x = min_index.x; x <= max_index.x; ++x) {
      for (int y = min_index.y; y <= max_index.y; ++y) {
        for (int z = min_index.z; z <= max_index.z; ++z) {
          const GridIndex index{x, y, z};
          const Eigen::Vector3d point = to_point(index);
          // Distance from the planning point to the occupied leaf's axis-aligned cube.
          const Eigen::Vector3d distance =
            (point - center).cwiseAbs() - Eigen::Vector3d::Constant(leaf_half_size);
          if (distance.cwiseMax(0.0).norm() <= config_.collision_radius + edge_margin) {
            blocked[flatten(index)] = 1U;
          }
        }
      }
    }
  }

  // Unknown-space checks are also cached once when that conservative mode is requested. The
  // normal underwater-survey mode treats unseen water as traversable and skips this entire pass.
  if (config_.unknown_is_occupied) {
    for (int z = 0; z < dimensions.z; ++z) {
      for (int y = 0; y < dimensions.y; ++y) {
        for (int x = 0; x < dimensions.x; ++x) {
          const GridIndex index{x, y, z};
          const std::uint32_t cell = flatten(index);
          if (blocked[cell] == 0U && !isStateFree(tree, to_point(index))) {
            blocked[cell] = 1U;
          }
        }
      }
    }
  }

  const std::uint32_t start_cell = flatten(start_index);
  const std::uint32_t goal_cell = flatten(goal_index);
  if (blocked[start_cell] != 0U || blocked[goal_cell] != 0U) {
    return {};
  }

  // Collision-check a segment against the conservative inflated lattice. This replaces repeated
  // spherical octree probes during direct-path tests and path smoothing.
  const auto line_is_free = [&](const Eigen::Vector3d & from, const Eigen::Vector3d & to) {
      const double distance = (to - from).norm();
      const double step = 0.5 * config_.planning_resolution;
      const int samples = std::max(1, static_cast<int>(std::ceil(distance / step)));
      for (int i = 0; i <= samples; ++i) {
        const double fraction = static_cast<double>(i) / static_cast<double>(samples);
        const GridIndex index = to_index(from + fraction * (to - from));
        if (!index_in_bounds(index) || blocked[flatten(index)] != 0U) {
          return false;
        }
      }
      return true;
    };
  if (line_is_free(start, goal)) {
    return {start, goal};
  }

  std::vector<float> cost(cell_count, std::numeric_limits<float>::infinity());
  std::vector<std::uint32_t> parent(
    cell_count, std::numeric_limits<std::uint32_t>::max());
  std::vector<std::uint8_t> closed(cell_count, 0U);
  std::vector<QueueEntry> open_storage;
  open_storage.reserve(std::min(cell_count, config_.max_expansions));
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, GreaterPriority> open(
    GreaterPriority{}, std::move(open_storage));

  const auto heuristic = [&](const GridIndex & index) {
      const double dx = static_cast<double>(goal_index.x - index.x);
      const double dy = static_cast<double>(goal_index.y - index.y);
      const double dz = static_cast<double>(goal_index.z - index.z);
      return static_cast<float>(
        config_.planning_resolution * std::sqrt(dx * dx + dy * dy + dz * dz));
    };

  cost[start_cell] = 0.0F;
  open.push({heuristic(start_index), start_cell});

  std::size_t expansions = 0U;
  bool found = false;
  while (!open.empty() && expansions < config_.max_expansions) {
    if ((expansions & 0x7fU) == 0U && cancel_checker && cancel_checker()) {
      return {};
    }

    const std::uint32_t current_cell = open.top().cell;
    open.pop();
    if (closed[current_cell] != 0U) {
      continue;
    }
    if (current_cell == goal_cell) {
      found = true;
      break;
    }
    closed[current_cell] = 1U;
    ++expansions;
    const GridIndex current = expand(current_cell);

    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const GridIndex next{current.x + dx, current.y + dy, current.z + dz};
          if (!index_in_bounds(next)) {
            continue;
          }
          const std::uint32_t next_cell = flatten(next);
          if (closed[next_cell] != 0U || blocked[next_cell] != 0U) {
            continue;
          }

          const float edge_cost = static_cast<float>(
            config_.planning_resolution * std::sqrt(
              static_cast<double>(dx * dx + dy * dy + dz * dz)));
          const float candidate_cost = cost[current_cell] + edge_cost;
          if (candidate_cost >= cost[next_cell]) {
            continue;
          }
          cost[next_cell] = candidate_cost;
          parent[next_cell] = current_cell;
          open.push({candidate_cost + heuristic(next), next_cell});
        }
      }
    }
  }

  if (!found) {
    return {};
  }

  std::vector<Eigen::Vector3d> raw_path;
  for (std::uint32_t cell = goal_cell; cell != start_cell; cell = parent[cell]) {
    if (cell == std::numeric_limits<std::uint32_t>::max()) {
      return {};
    }
    raw_path.push_back(to_point(expand(cell)));
  }
  raw_path.push_back(to_point(start_index));
  std::reverse(raw_path.begin(), raw_path.end());
  raw_path.front() = start;
  raw_path.back() = goal;

  // Greedily remove grid stair-steps using the same conservative collision cache.
  std::vector<Eigen::Vector3d> simplified;
  std::size_t anchor = 0U;
  simplified.push_back(raw_path.front());
  while (anchor + 1U < raw_path.size()) {
    std::size_t next = raw_path.size() - 1U;
    while (next > anchor + 1U && !line_is_free(raw_path[anchor], raw_path[next])) {
      --next;
    }
    simplified.push_back(raw_path[next]);
    anchor = next;
  }
  return simplified;
}

}  // namespace rov_nav2_plugins
