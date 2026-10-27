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

#include "rov_nav2_plugins/costmap/costmap_math.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace rov_nav2_plugins
{
namespace
{

constexpr double kInfinity = std::numeric_limits<double>::infinity();
constexpr double kClear = -kInfinity;

/**
 * @brief One-dimensional squared distance transform, for squared distances weighted by `weight_sq`.
 *
 * Sets `out[q]` to the minimum over p of `weight_sq * (q - p)^2 + in[p]`, the lower envelope of a parabola
 * rooted at each finite `in[p]` (Felzenszwalb and Huttenlocher). Infinite entries root no parabola, and
 * all of `out` is infinite if `in` has no finite entry. `sites` and `bounds` are scratch space.
 */
void distance_transform_1d(
  const std::vector<double> & in,
  std::vector<double> & out,
  double weight_sq,
  std::vector<int> & sites,
  std::vector<double> & bounds)
{
  const int n = static_cast<int>(in.size());
  // sites[0..k] are the roots of the parabolas in the lower envelope; parabola k is lowest from bounds[k]
  int k = -1;
  for (int q = 0; q < n; ++q) {
    if (!std::isfinite(in[q])) {
      continue;
    }
    double s = -kInfinity;
    while (k >= 0) {
      const int p = sites[k];
      s = ((in[q] + weight_sq * q * q) - (in[p] + weight_sq * p * p)) / (2.0 * weight_sq * (q - p));
      if (s > bounds[k]) {
        break;
      }
      --k;
    }
    ++k;
    sites[k] = q;
    bounds[k] = k == 0 ? -kInfinity : s;
  }

  if (k < 0) {
    std::fill(out.begin(), out.end(), kInfinity);
    return;
  }
  bounds[k + 1] = kInfinity;
  int j = 0;
  for (int q = 0; q < n; ++q) {
    while (bounds[j + 1] < q) {
      ++j;
    }
    const double offset = q - sites[j];
    out[q] = weight_sq * offset * offset + in[sites[j]];
  }
}

/// Apply distance_transform_1d() in place to every line of `values` along `axis`.
void distance_transform_axis(const GridGeometry & geometry, std::vector<double> & values, int axis, double weight_sq)
{
  const int n = geometry.size[axis];
  const std::size_t stride = axis == 0 ? 1 : axis == 1 ? geometry.size.x() : geometry.size.x() * geometry.size.y();
  std::vector<double> in(n);
  std::vector<double> out(n);
  std::vector<int> sites(n);
  std::vector<double> bounds(n + 1);

  // Each line starts at a voxel whose coordinate along `axis` is zero
  Eigen::Vector3i start{Eigen::Vector3i::Zero()};
  const int u = (axis + 1) % 3;
  const int v = (axis + 2) % 3;
  for (start[v] = 0; start[v] < geometry.size[v]; ++start[v]) {
    for (start[u] = 0; start[u] < geometry.size[u]; ++start[u]) {
      const std::size_t first = geometry.index(start);
      for (int q = 0; q < n; ++q) {
        in[q] = values[first + q * stride];
      }
      distance_transform_1d(in, out, weight_sq, sites, bounds);
      for (int q = 0; q < n; ++q) {
        values[first + q * stride] = out[q];
      }
    }
  }
}

}  // namespace

std::size_t GridGeometry::count() const
{
  return static_cast<std::size_t>(size.x()) * size.y() * size.z();
}

bool GridGeometry::contains(const Eigen::Vector3i & voxel) const
{
  return (voxel.array() >= 0).all() && (voxel.array() < size.array()).all();
}

std::size_t GridGeometry::index(const Eigen::Vector3i & voxel) const
{
  return (static_cast<std::size_t>(voxel.z()) * size.y() + voxel.y()) * size.x() + voxel.x();
}

std::optional<Eigen::Vector3i> GridGeometry::voxel_at(const Eigen::Vector3d & point) const
{
  if (!point.allFinite()) {
    return std::nullopt;
  }
  const Eigen::Vector3d coordinates = ((point - origin) / resolution).array().floor();
  // Compare as doubles before converting, so a far-away point cannot overflow an int
  if ((coordinates.array() < 0.0).any() || (coordinates.array() >= size.cast<double>().array()).any()) {
    return std::nullopt;
  }
  return coordinates.cast<int>();
}

Eigen::Vector3d GridGeometry::center(const Eigen::Vector3i & voxel) const
{
  return origin + resolution * (voxel.cast<double>().array() + 0.5).matrix();
}

std::vector<Eigen::Vector3i> traverse(
  const GridGeometry & geometry, const Eigen::Vector3d & from, const Eigen::Vector3d & to)
{
  // Work in voxel units, with the grid spanning [0, size) on each axis
  const Eigen::Vector3d start = (from - geometry.origin) / geometry.resolution;
  const Eigen::Vector3d delta = (to - geometry.origin) / geometry.resolution - start;
  if (!start.allFinite() || !delta.allFinite()) {
    return {};
  }

  // Clip the segment, start + t * delta for t in [0, 1], to the grid's box (the slab method)
  double t_enter = 0.0;
  double t_exit = 1.0;
  for (int axis = 0; axis < 3; ++axis) {
    if (delta[axis] == 0.0) {
      if (start[axis] < 0.0 || start[axis] >= geometry.size[axis]) {
        return {};
      }
      continue;
    }
    double t0 = -start[axis] / delta[axis];
    double t1 = (geometry.size[axis] - start[axis]) / delta[axis];
    if (t0 > t1) {
      std::swap(t0, t1);
    }
    t_enter = std::max(t_enter, t0);
    t_exit = std::min(t_exit, t1);
  }
  if (t_enter > t_exit) {
    return {};
  }

  // The clipped ends can sit exactly on the grid's far faces, so clamp them into the grid
  const auto voxel_of = [&geometry](const Eigen::Vector3d & point) {
    const Eigen::Vector3d upper = (geometry.size.array() - 1).cast<double>();
    return point.array().floor().max(0.0).min(upper.array()).cast<int>().matrix().eval();
  };
  const Eigen::Vector3d entry = start + t_enter * delta;
  Eigen::Vector3i voxel = voxel_of(entry);
  const Eigen::Vector3i last = voxel_of(start + t_exit * delta);

  // Along each axis: the direction of travel, the t at which the segment crosses into the next voxel,
  // and the t it takes to cross a whole voxel
  Eigen::Vector3i step{Eigen::Vector3i::Zero()};
  Eigen::Vector3d t_next{Eigen::Vector3d::Constant(kInfinity)};
  Eigen::Vector3d t_voxel{Eigen::Vector3d::Constant(kInfinity)};
  for (int axis = 0; axis < 3; ++axis) {
    if (voxel[axis] == last[axis]) {
      continue;
    }
    step[axis] = last[axis] > voxel[axis] ? 1 : -1;
    t_voxel[axis] = 1.0 / std::abs(delta[axis]);
    const double boundary = voxel[axis] + (step[axis] > 0 ? 1 : 0);
    t_next[axis] = t_enter + (boundary - entry[axis]) / delta[axis];
  }

  // Exactly one face crossing per voxel of Manhattan distance to the last voxel. An axis stops once it
  // reaches the last voxel, so rounding can never carry the walk past it.
  const int crossings = (last - voxel).cwiseAbs().sum();
  std::vector<Eigen::Vector3i> voxels{voxel};
  voxels.reserve(crossings + 1);
  for (int n = 0; n < crossings; ++n) {
    int axis = 0;
    t_next.minCoeff(&axis);
    voxel[axis] += step[axis];
    t_next[axis] = voxel[axis] == last[axis] ? kInfinity : t_next[axis] + t_voxel[axis];
    voxels.push_back(voxel);
  }
  return voxels;
}

VoxelGrid3D::VoxelGrid3D(const GridGeometry & geometry)
: geometry_(geometry)
{
  if (geometry_.resolution <= 0.0 || (geometry_.size.array() <= 0).any()) {
    throw std::invalid_argument("The voxel grid needs a positive resolution and at least one voxel on each axis");
  }
  marked_at_.assign(geometry_.count(), kClear);
}

const GridGeometry & VoxelGrid3D::geometry() const
{
  return geometry_;
}

void VoxelGrid3D::insert(
  const Eigen::Vector3d & origin, const std::vector<Eigen::Vector3d> & points, double stamp, double max_range)
{
  if (!origin.allFinite()) {
    return;
  }
  std::vector<Eigen::Vector3d> returns;
  returns.reserve(points.size());
  for (const auto & point : points) {
    if (point.allFinite() && (point - origin).norm() <= max_range) {
      returns.push_back(point);
    }
  }

  for (const auto & point : returns) {
    for (const auto & voxel : traverse(geometry_, origin, point)) {
      marked_at_[geometry_.index(voxel)] = kClear;
    }
  }
  for (const auto & point : returns) {
    if (const auto voxel = geometry_.voxel_at(point)) {
      marked_at_[geometry_.index(*voxel)] = stamp;
    }
  }
}

std::size_t VoxelGrid3D::decay(double now, double max_age)
{
  std::size_t cleared = 0;
  for (double & marked_at : marked_at_) {
    if (marked_at != kClear && now - marked_at > max_age) {
      marked_at = kClear;
      ++cleared;
    }
  }
  return cleared;
}

void VoxelGrid3D::clear()
{
  std::fill(marked_at_.begin(), marked_at_.end(), kClear);
}

bool VoxelGrid3D::occupied(const Eigen::Vector3i & voxel) const
{
  return marked_at_[geometry_.index(voxel)] != kClear;
}

std::uint8_t inflated_cost(double distance, const InflationConfig & config)
{
  if (distance <= 0.0) {
    return kLethalCost;
  }
  if (distance <= config.inscribed_radius) {
    return kInscribedCost;
  }
  if (distance > config.inflation_radius) {
    return kFreeCost;
  }
  const double factor = std::exp(-config.cost_scaling_factor * (distance - config.inscribed_radius));
  return static_cast<std::uint8_t>((kInscribedCost - 1) * factor);
}

CostVolume::CostVolume(const GridGeometry & geometry, std::vector<std::uint8_t> costs)
: geometry_(geometry)
, costs_(std::move(costs))
{
  if (costs_.size() != geometry_.count()) {
    throw std::invalid_argument("A cost volume needs one cost per voxel");
  }
  const int size_x = geometry_.size.x();
  const int size_y = geometry_.size.y();
  column_min_.assign(static_cast<std::size_t>(size_x) * size_y, kLethalCost);
  for (int k = 0; k < geometry_.size.z(); ++k) {
    for (int j = 0; j < size_y; ++j) {
      for (int i = 0; i < size_x; ++i) {
        auto & lowest = column_min_[static_cast<std::size_t>(j) * size_x + i];
        lowest = std::min(lowest, cost({i, j, k}));
      }
    }
  }
}

const GridGeometry & CostVolume::geometry() const
{
  return geometry_;
}

std::uint8_t CostVolume::cost(const Eigen::Vector3i & voxel) const
{
  return costs_[geometry_.index(voxel)];
}

std::uint8_t CostVolume::column_min(int i, int j) const
{
  return column_min_[static_cast<std::size_t>(j) * geometry_.size.x() + i];
}

CostVolume inflate(const VoxelGrid3D & grid, const InflationConfig & config)
{
  if (
    config.inscribed_radius <= 0.0 || config.half_height <= 0.0 || config.cost_scaling_factor <= 0.0 ||
    config.inflation_radius < config.inscribed_radius)
  {
    throw std::invalid_argument(
      "Inflation needs a positive inscribed radius, half-height and cost scaling factor, and an inflation radius "
      "no less than the inscribed radius");
  }

  // Squared distance to the nearest occupied voxel, in voxel units, with z scaled to make the vehicle round
  const GridGeometry & geometry = grid.geometry();
  std::vector<double> distance_sq(geometry.count(), kInfinity);
  Eigen::Vector3i voxel;
  for (voxel.z() = 0; voxel.z() < geometry.size.z(); ++voxel.z()) {
    for (voxel.y() = 0; voxel.y() < geometry.size.y(); ++voxel.y()) {
      for (voxel.x() = 0; voxel.x() < geometry.size.x(); ++voxel.x()) {
        if (grid.occupied(voxel)) {
          distance_sq[geometry.index(voxel)] = 0.0;
        }
      }
    }
  }
  const double z_scale = config.inscribed_radius / config.half_height;
  distance_transform_axis(geometry, distance_sq, 0, 1.0);
  distance_transform_axis(geometry, distance_sq, 1, 1.0);
  distance_transform_axis(geometry, distance_sq, 2, z_scale * z_scale);

  std::vector<std::uint8_t> costs(geometry.count());
  for (std::size_t n = 0; n < costs.size(); ++n) {
    costs[n] = inflated_cost(std::sqrt(distance_sq[n]) * geometry.resolution, config);
  }
  return CostVolume(geometry, std::move(costs));
}

}  // namespace rov_nav2_plugins
