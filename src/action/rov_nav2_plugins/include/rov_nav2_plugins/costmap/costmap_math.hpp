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

#pragma once

#include <cstddef>
#include <cstdint>
#include <Eigen/Core>
#include <optional>
#include <vector>

// VoxelLayer3D's 3D occupancy and cost math, independent of Nav2 and ROS communication.

namespace rov_nav2_plugins
{
// --- Costs, with the same values as Nav2's cost_values.hpp ---

constexpr std::uint8_t kFreeCost = 0;
constexpr std::uint8_t kInscribedCost = 253;  // the vehicle's center here puts its body in an obstacle
constexpr std::uint8_t kLethalCost = 254;  // an obstacle

// --- Grid geometry ---

/**
 * @brief The extent of a 3D grid of cubic voxels.
 *
 * Voxel (i, j, k) spans `origin + resolution * [i, i + 1) x [j, j + 1) x [k, k + 1)`. Voxels are stored
 * with x varying fastest, then y, then z.
 */
struct GridGeometry
{
  Eigen::Vector3d origin{Eigen::Vector3d::Zero()};  // m, the minimum corner of voxel (0, 0, 0)
  double resolution{0.1};  // m, the edge of a voxel
  Eigen::Vector3i size{Eigen::Vector3i::Zero()};  // voxels along x, y and z

  std::size_t count() const;

  bool contains(const Eigen::Vector3i & voxel) const;

  /// Storage index of `voxel`, which must be in the grid.
  std::size_t index(const Eigen::Vector3i & voxel) const;

  /// The voxel containing `point`, or nothing if `point` is outside the grid.
  std::optional<Eigen::Vector3i> voxel_at(const Eigen::Vector3d & point) const;

  Eigen::Vector3d center(const Eigen::Vector3i & voxel) const;
};

/**
 * @brief The voxels the segment from `from` to `to` passes through, in order, clipped to the grid.
 *
 * A 3D digital differential analyzer (Amanatides and Woo): each step moves to the face-adjacent voxel
 * that the segment enters next. Consecutive voxels therefore share a face, and the list ends with the
 * voxel containing `to`, if that is in the grid. Empty if the segment misses the grid.
 */
std::vector<Eigen::Vector3i> traverse(
  const GridGeometry & geometry, const Eigen::Vector3d & from, const Eigen::Vector3d & to);

// --- Occupancy ---

/**
 * @brief 3D occupancy from range returns, each voxel remembering when it was last seen occupied.
 *
 * Every return is an obstacle, the floor included; nothing is filtered by height.
 */
class VoxelGrid3D
{
public:
  /// @throws std::invalid_argument if the resolution is not positive or the grid is empty
  explicit VoxelGrid3D(const GridGeometry & geometry);

  const GridGeometry & geometry() const;

  /**
   * @brief Add one scan, seen from `origin` at time `stamp` (s).
   *
   * The space between the sensor and each return is clear, so the voxels on each ray are cleared, then
   * the voxel of each return is marked. All rays are cleared before any return is marked, so one
   * return's ray never erases another return from the same scan, such as the floor beside it.
   *
   * Returns farther than `max_range` from `origin`, and returns that are not finite, are ignored.
   */
  void insert(
    const Eigen::Vector3d & origin, const std::vector<Eigen::Vector3d> & points, double stamp, double max_range);

  /// Clear every voxel last marked more than `max_age` (s) before `now` (s). Returns how many were cleared.
  std::size_t decay(double now, double max_age);

  /// Clear every voxel.
  void clear();

  bool occupied(const Eigen::Vector3i & voxel) const;

private:
  GridGeometry geometry_;
  std::vector<double> marked_at_;  // s, the time each voxel was last marked; -infinity if clear
};

// --- Inflation ---

struct InflationConfig
{
  double inscribed_radius{0.0};  // m, the vehicle's horizontal half-width
  double half_height{0.0};  // m, the vehicle's vertical half-height
  double inflation_radius{0.0};  // m, horizontal distance from obstacles at which cost reaches zero
  double cost_scaling_factor{0.0};  // 1/m, how fast cost falls off beyond the inscribed radius
};

/**
 * @brief The cost of each voxel for the vehicle's center, from its distance to the nearest obstacle.
 *
 * The vehicle is wider than it is tall, so distance is measured in an ellipsoidal metric: vertical
 * distance is scaled by `inscribed_radius / half_height`, making the vehicle's footprint a sphere of
 * radius `inscribed_radius`. In that metric, the cost follows Nav2's InflationLayer: lethal at an
 * obstacle, inscribed within `inscribed_radius` of one, then
 * `(kInscribedCost - 1) * exp(-cost_scaling_factor * (d - inscribed_radius))` out to `inflation_radius`,
 * and free beyond.
 */
std::uint8_t inflated_cost(double distance, const InflationConfig & config);

/// Immutable 3D costs over a grid, as computed by inflate().
class CostVolume
{
public:
  /// @throws std::invalid_argument if there is not one cost per voxel
  CostVolume(const GridGeometry & geometry, std::vector<std::uint8_t> costs);

  const GridGeometry & geometry() const;

  /// Cost of `voxel`, which must be in the grid.
  std::uint8_t cost(const Eigen::Vector3i & voxel) const;

  /// Lowest cost in column (`i`, `j`), which must be in the grid: the cost of passing there at the best depth.
  std::uint8_t column_min(int i, int j) const;

private:
  GridGeometry geometry_;
  std::vector<std::uint8_t> costs_;
  std::vector<std::uint8_t> column_min_;
};

/**
 * @brief Inflate the occupied voxels of `grid` in 3D.
 *
 * Distances come from an exact Euclidean distance transform (Felzenszwalb and Huttenlocher), one pass
 * per axis, so the cost is linear in the number of voxels.
 *
 * @throws std::invalid_argument if a radius, the half-height or the scaling factor is not positive, or the
 *   inflation radius is less than the inscribed radius
 */
CostVolume inflate(const VoxelGrid3D & grid, const InflationConfig & config);

}  // namespace rov_nav2_plugins
