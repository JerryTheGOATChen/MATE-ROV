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
#include <cstddef>
#include <cstdint>
#include <Eigen/Core>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "rov_nav2_plugins/costmap/costmap_math.hpp"

using rov_nav2_plugins::CostVolume;
using rov_nav2_plugins::GridGeometry;
using rov_nav2_plugins::inflate;
using rov_nav2_plugins::inflated_cost;
using rov_nav2_plugins::InflationConfig;
using rov_nav2_plugins::kFreeCost;
using rov_nav2_plugins::kInscribedCost;
using rov_nav2_plugins::kLethalCost;
using rov_nav2_plugins::traverse;
using rov_nav2_plugins::VoxelGrid3D;

namespace
{

/// A 2 x 1 x 1 m grid of 0.1 m voxels, with voxel (0, 0, 0) at the origin.
GridGeometry pool_geometry()
{
  GridGeometry geometry;
  geometry.resolution = 0.1;
  geometry.size = {20, 10, 10};
  return geometry;
}

/// Mark the voxel containing `point`, as a return seen from inside that voxel.
void mark(VoxelGrid3D & grid, const Eigen::Vector3d & point, double stamp = 0.0)
{
  grid.insert(point, {point}, stamp, 1.0);
}

InflationConfig vehicle()
{
  InflationConfig config;
  config.inscribed_radius = 0.3;
  config.half_height = 0.15;
  config.inflation_radius = 0.6;
  config.cost_scaling_factor = 3.0;
  return config;
}

}  // namespace

TEST_CASE("GridGeometry maps points to voxels and back", "[costmap_math]")
{
  GridGeometry geometry = pool_geometry();
  geometry.origin = {-1.0, -0.5, -1.0};

  REQUIRE(geometry.count() == 2000);
  REQUIRE(*geometry.voxel_at({-1.0, -0.5, -1.0}) == Eigen::Vector3i(0, 0, 0));
  REQUIRE(*geometry.voxel_at({0.05, 0.0, -0.01}) == Eigen::Vector3i(10, 5, 9));
  REQUIRE((geometry.center({10, 5, 9}) - Eigen::Vector3d(0.05, 0.05, -0.05)).norm() < 1e-9);

  SECTION("Points outside the grid, or not finite, have no voxel")
  {
    REQUIRE_FALSE(geometry.voxel_at({1.0, 0.0, -0.5}));  // the far face is outside
    REQUIRE_FALSE(geometry.voxel_at({0.0, 0.0, 0.01}));
    REQUIRE_FALSE(geometry.voxel_at({1e30, 0.0, -0.5}));
    REQUIRE_FALSE(geometry.voxel_at({std::numeric_limits<double>::quiet_NaN(), 0.0, -0.5}));
  }
}

TEST_CASE("traverse visits every voxel along a segment, in order", "[costmap_math]")
{
  const GridGeometry geometry = pool_geometry();

  SECTION("Along an axis")
  {
    const auto voxels = traverse(geometry, {0.05, 0.05, 0.05}, {0.55, 0.05, 0.05});
    REQUIRE(voxels.size() == 6);
    for (int i = 0; i < 6; ++i) {
      REQUIRE(voxels[i] == Eigen::Vector3i(i, 0, 0));
    }
  }

  SECTION("Diagonally, one face at a time, ending at the end's voxel")
  {
    const Eigen::Vector3d from{0.12, 0.03, 0.91};
    const Eigen::Vector3d to{1.77, 0.86, 0.04};
    const auto voxels = traverse(geometry, from, to);

    REQUIRE(voxels.front() == *geometry.voxel_at(from));
    REQUIRE(voxels.back() == *geometry.voxel_at(to));
    for (std::size_t n = 1; n < voxels.size(); ++n) {
      REQUIRE((voxels[n] - voxels[n - 1]).cwiseAbs().sum() == 1);
    }
    // The segment passes through every voxel visited: the center of each is within half a diagonal of it
    const Eigen::Vector3d direction = (to - from).normalized();
    for (const auto & voxel : voxels) {
      const Eigen::Vector3d offset = geometry.center(voxel) - from;
      const double off_line = (offset - offset.dot(direction) * direction).norm();
      REQUIRE(off_line <= geometry.resolution * std::sqrt(3.0) / 2.0 + 1e-9);
    }
  }

  SECTION("Clipped to the grid")
  {
    const auto voxels = traverse(geometry, {-1.0, 0.05, 0.05}, {0.25, 0.05, 0.05});
    REQUIRE(voxels.size() == 3);
    REQUIRE(voxels.front() == Eigen::Vector3i(0, 0, 0));
    REQUIRE(voxels.back() == Eigen::Vector3i(2, 0, 0));

    const auto through = traverse(geometry, {0.05, 0.05, -1.0}, {0.05, 0.05, 2.0});
    REQUIRE(through.size() == 10);
  }

  SECTION("Empty when the segment misses the grid")
  {
    REQUIRE(traverse(geometry, {-1.0, 0.05, 0.05}, {-0.5, 0.05, 0.05}).empty());
    REQUIRE(traverse(geometry, {0.05, 2.0, 0.05}, {1.0, 2.0, 0.05}).empty());
  }
}

TEST_CASE("VoxelGrid3D clears the space before a return and marks the return", "[costmap_math]")
{
  VoxelGrid3D grid(pool_geometry());
  const Eigen::Vector3d sonar{0.05, 0.55, 0.55};

  // An obstacle seen earlier, now in front of a farther return
  mark(grid, {0.95, 0.55, 0.55});
  REQUIRE(grid.occupied({9, 5, 5}));

  grid.insert(sonar, {{1.55, 0.55, 0.55}}, 1.0, 8.0);

  REQUIRE_FALSE(grid.occupied({9, 5, 5}));
  REQUIRE(grid.occupied({15, 5, 5}));

  SECTION("Returns beyond the obstacle range, or not finite, are ignored")
  {
    grid.insert(sonar, {{1.95, 0.55, 0.55}, {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}}, 2.0, 1.5);
    REQUIRE(grid.occupied({15, 5, 5}));
    REQUIRE_FALSE(grid.occupied({19, 5, 5}));
  }

  SECTION("One return's ray does not erase another return from the same scan")
  {
    // The second return lies on the line to the first
    grid.insert(sonar, {{1.55, 0.55, 0.55}, {0.75, 0.55, 0.55}}, 2.0, 8.0);
    REQUIRE(grid.occupied({15, 5, 5}));
    REQUIRE(grid.occupied({7, 5, 5}));
  }

  SECTION("Rays grazing the floor keep the floor the same scan sees")
  {
    // A floor along z = 0, seen at a shallow angle from 0.2 m above it
    std::vector<Eigen::Vector3d> floor;
    for (int i = 5; i < 20; ++i) {
      floor.emplace_back(0.05 + 0.1 * i, 0.55, 0.05);
    }
    grid.insert({0.05, 0.55, 0.25}, floor, 2.0, 8.0);
    for (int i = 5; i < 20; ++i) {
      REQUIRE(grid.occupied({i, 5, 0}));
    }
  }
}

TEST_CASE("VoxelGrid3D forgets voxels not seen for longer than the decay time", "[costmap_math]")
{
  VoxelGrid3D grid(pool_geometry());
  mark(grid, {0.05, 0.05, 0.05}, 10.0);
  mark(grid, {0.55, 0.05, 0.05}, 30.0);

  REQUIRE(grid.decay(60.0, 40.0) == 1);
  REQUIRE_FALSE(grid.occupied({0, 0, 0}));
  REQUIRE(grid.occupied({5, 0, 0}));
  REQUIRE(grid.decay(60.0, 40.0) == 0);

  grid.clear();
  REQUIRE_FALSE(grid.occupied({5, 0, 0}));
}

TEST_CASE("VoxelGrid3D needs a grid", "[costmap_math]")
{
  GridGeometry geometry = pool_geometry();
  geometry.size.z() = 0;
  REQUIRE_THROWS_AS(VoxelGrid3D(geometry), std::invalid_argument);
}

TEST_CASE("inflated_cost follows Nav2's InflationLayer", "[costmap_math]")
{
  const InflationConfig config = vehicle();

  REQUIRE(inflated_cost(0.0, config) == kLethalCost);
  REQUIRE(inflated_cost(0.1, config) == kInscribedCost);
  REQUIRE(inflated_cost(0.3, config) == kInscribedCost);
  // Nav2: (INSCRIBED_INFLATED_OBSTACLE - 1) * exp(-cost_scaling_factor * (distance - inscribed_radius))
  REQUIRE(inflated_cost(0.4, config) == static_cast<std::uint8_t>(252.0 * std::exp(-3.0 * 0.1)));
  REQUIRE(inflated_cost(0.6, config) == static_cast<std::uint8_t>(252.0 * std::exp(-3.0 * 0.3)));
  REQUIRE(inflated_cost(0.61, config) == kFreeCost);
}

TEST_CASE("inflate matches a brute-force search for the nearest obstacle", "[costmap_math]")
{
  GridGeometry geometry;
  geometry.resolution = 0.1;
  geometry.size = {13, 9, 11};
  const InflationConfig config = vehicle();
  const double z_scale = config.inscribed_radius / config.half_height;

  std::mt19937 random(42);
  for (int trial = 0; trial < 10; ++trial) {
    VoxelGrid3D grid(geometry);
    std::vector<Eigen::Vector3i> obstacles;
    std::uniform_int_distribution<int> count(1, 6);
    for (int n = count(random); n > 0; --n) {
      const Eigen::Vector3i voxel{
        std::uniform_int_distribution<int>(0, geometry.size.x() - 1)(random),
        std::uniform_int_distribution<int>(0, geometry.size.y() - 1)(random),
        std::uniform_int_distribution<int>(0, geometry.size.z() - 1)(random)};
      mark(grid, geometry.center(voxel));
      obstacles.push_back(voxel);
    }

    const CostVolume volume = inflate(grid, config);

    Eigen::Vector3i voxel;
    for (voxel.z() = 0; voxel.z() < geometry.size.z(); ++voxel.z()) {
      for (voxel.y() = 0; voxel.y() < geometry.size.y(); ++voxel.y()) {
        for (voxel.x() = 0; voxel.x() < geometry.size.x(); ++voxel.x()) {
          double nearest = std::numeric_limits<double>::infinity();
          for (const auto & obstacle : obstacles) {
            Eigen::Vector3d offset = (voxel - obstacle).cast<double>() * geometry.resolution;
            offset.z() *= z_scale;
            nearest = std::min(nearest, offset.norm());
          }
          REQUIRE(volume.cost(voxel) == inflated_cost(nearest, config));
        }
      }
    }
  }
}

TEST_CASE(
  "inflate keeps the vehicle's half-height off the floor, and columns report their best depth", "[costmap_math]")
{
  VoxelGrid3D grid(pool_geometry());
  // The floor at the bottom of the grid, and a post through every depth at x = 1.05
  for (int i = 0; i < 20; ++i) {
    for (int j = 0; j < 10; ++j) {
      mark(grid, {0.05 + 0.1 * i, 0.05 + 0.1 * j, 0.05});
    }
  }
  for (int k = 0; k < 10; ++k) {
    mark(grid, {1.05, 0.55, 0.05 + 0.1 * k});
  }

  const InflationConfig config = vehicle();
  const CostVolume volume = inflate(grid, config);

  // Above the floor, cost falls off over the vehicle's half-height, not its half-width
  REQUIRE(volume.cost({3, 5, 0}) == kLethalCost);
  REQUIRE(volume.cost({3, 5, 1}) == kInscribedCost);  // 0.1 m up
  REQUIRE(volume.cost({3, 5, 2}) < kInscribedCost);  // 0.2 m up, beyond the 0.15 m half-height
  REQUIRE(volume.cost({3, 5, 4}) == kFreeCost);  // 0.4 m up, beyond 0.6 m horizontal-equivalent

  // In 2D, only the post's column is blocked at every depth
  REQUIRE(volume.column_min(10, 5) == kLethalCost);
  REQUIRE(volume.column_min(12, 5) == kInscribedCost);
  REQUIRE(volume.column_min(3, 5) == kFreeCost);
}

TEST_CASE("inflate rejects a vehicle with no size", "[costmap_math]")
{
  VoxelGrid3D grid(pool_geometry());
  InflationConfig config = vehicle();
  config.half_height = 0.0;
  REQUIRE_THROWS_AS(inflate(grid, config), std::invalid_argument);
}
