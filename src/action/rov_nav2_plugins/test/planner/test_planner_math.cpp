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
#include <Eigen/Geometry>
#include <limits>
#include <queue>
#include <random>
#include <stdexcept>
#include <utility>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "angles/angles.h"
#include "rov_nav2_plugins/controller/controller_math.hpp"
#include "rov_nav2_plugins/costmap/costmap_math.hpp"
#include "rov_nav2_plugins/planner/planner_math.hpp"

using Catch::Matchers::WithinAbs;
using rov_nav2_plugins::add_yaw;
using rov_nav2_plugins::BodyVelocity;
using rov_nav2_plugins::CostVolume;
using rov_nav2_plugins::GridGeometry;
using rov_nav2_plugins::inflate;
using rov_nav2_plugins::InflationConfig;
using rov_nav2_plugins::kFreeCost;
using rov_nav2_plugins::kInscribedCost;
using rov_nav2_plugins::kLethalCost;
using rov_nav2_plugins::Plan3D;
using rov_nav2_plugins::plan_through_voxels;
using rov_nav2_plugins::PlannerConfig;
using rov_nav2_plugins::PlanStatus;
using rov_nav2_plugins::Pose4D;
using rov_nav2_plugins::Pursuit;
using rov_nav2_plugins::PursuitConfig;
using rov_nav2_plugins::traverse;
using rov_nav2_plugins::VoxelGrid3D;
using rov_nav2_plugins::YawMode;

namespace
{

constexpr double kTolerance = 1e-9;
// The route keeps the vehicle's half-width or half-height from obstacles, measured between voxel centers, so a
// segment can pass up to a voxel closer to an obstacle's face than that
constexpr double kVoxel = 0.1;  // m

bool never_canceled()
{
  return false;
}

/// The ROV in action_bringup's nav2.yaml.
InflationConfig rov()
{
  InflationConfig config;
  config.inscribed_radius = 0.35;
  config.half_height = 0.2;
  config.inflation_radius = 1.0;
  config.cost_scaling_factor = 3.0;
  return config;
}

/// A tank of 0.1 m voxels, `size` m long, wide and deep, with the water surface at z = 0 and obstacles added as
/// boxes. The sensor is taken to have seen all of it unless a test says otherwise.
struct Tank
{
  explicit Tank(const Eigen::Vector3d & size)
  : grid(make_geometry(size))
  {}

  static GridGeometry make_geometry(const Eigen::Vector3d & size)
  {
    GridGeometry geometry;
    geometry.resolution = 0.1;
    geometry.origin = {0.0, 0.0, -size.z()};
    geometry.size = (size / geometry.resolution).array().round().cast<int>();
    return geometry;
  }

  const GridGeometry & geometry() const
  {
    return grid.geometry();
  }

  /// Mark every voxel whose center is in the box from `low` to `high` as an obstacle.
  void add_box(const Eigen::Vector3d & low, const Eigen::Vector3d & high)
  {
    Eigen::Vector3i voxel;
    for (voxel.z() = 0; voxel.z() < geometry().size.z(); ++voxel.z()) {
      for (voxel.y() = 0; voxel.y() < geometry().size.y(); ++voxel.y()) {
        for (voxel.x() = 0; voxel.x() < geometry().size.x(); ++voxel.x()) {
          const Eigen::Vector3d center = geometry().center(voxel);
          if ((center.array() >= low.array()).all() && (center.array() <= high.array()).all()) {
            grid.insert(center, {center}, 0.0, 1.0);
          }
        }
      }
    }
  }

  void add_floor()
  {
    add_box(geometry().origin, geometry().origin + Eigen::Vector3d(100.0, 100.0, 0.1));
  }

  /// The inflated costs, with each voxel known where `known` says so.
  template <typename Known>
  CostVolume volume(Known known) const
  {
    const CostVolume inflated = inflate(grid, rov());
    std::vector<std::uint8_t> costs;
    std::vector<bool> known_flags;
    Eigen::Vector3i voxel;
    for (voxel.z() = 0; voxel.z() < geometry().size.z(); ++voxel.z()) {
      for (voxel.y() = 0; voxel.y() < geometry().size.y(); ++voxel.y()) {
        for (voxel.x() = 0; voxel.x() < geometry().size.x(); ++voxel.x()) {
          costs.push_back(inflated.cost(voxel));
          known_flags.push_back(known(geometry().center(voxel)));
        }
      }
    }
    return CostVolume(geometry(), std::move(costs), std::move(known_flags));
  }

  CostVolume volume() const
  {
    return volume([](const Eigen::Vector3d &) { return true; });
  }

  VoxelGrid3D grid;
};

Plan3D plan(const CostVolume & volume, const Eigen::Vector3d & start, const Eigen::Vector3d & goal)
{
  return plan_through_voxels(volume, start, goal, PlannerConfig{}, never_canceled);
}

/// Every voxel each segment of `points` crosses is one the vehicle can be in without touching an obstacle.
bool clear_of_obstacles(const CostVolume & volume, const std::vector<Eigen::Vector3d> & points)
{
  for (std::size_t i = 1; i < points.size(); ++i) {
    for (const auto & voxel : traverse(volume.geometry(), points[i - 1], points[i])) {
      if (volume.cost(voxel) >= kInscribedCost) {
        return false;
      }
    }
  }
  return true;
}

double lowest(const std::vector<Eigen::Vector3d> & points)
{
  double z = std::numeric_limits<double>::infinity();
  for (const auto & point : points) {
    z = std::min(z, point.z());
  }
  return z;
}

/// Where the route crosses the plane x = `x`, interpolated along the segment that crosses it.
Eigen::Vector3d crossing(const std::vector<Eigen::Vector3d> & points, double x)
{
  for (std::size_t i = 1; i < points.size(); ++i) {
    const Eigen::Vector3d & a = points[i - 1];
    const Eigen::Vector3d & b = points[i];
    if ((a.x() - x) * (b.x() - x) <= 0.0 && a.x() != b.x()) {
      return a + (x - a.x()) / (b.x() - a.x()) * (b - a);
    }
  }
  throw std::logic_error("The route does not cross the plane");
}

/// Cheapest route cost from `start` to `goal` by Dijkstra's algorithm, with the step costs plan_through_voxels()
/// documents. Infinite if there is no route.
double cheapest_cost(
  const CostVolume & volume, const Eigen::Vector3i & start, const Eigen::Vector3i & goal, const PlannerConfig & config)
{
  const GridGeometry & geometry = volume.geometry();
  std::vector<double> best(geometry.count(), std::numeric_limits<double>::infinity());
  using Entry = std::pair<double, Eigen::Vector3i>;
  const auto later = [](const Entry & a, const Entry & b) { return a.first > b.first; };
  std::priority_queue<Entry, std::vector<Entry>, decltype(later)> queue(later);
  best[geometry.index(start)] = 0.0;
  queue.push({0.0, start});
  while (!queue.empty()) {
    const auto [cost, voxel] = queue.top();
    queue.pop();
    if (cost > best[geometry.index(voxel)]) {
      continue;
    }
    for (int dz = -1; dz <= 1; ++dz) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const Eigen::Vector3i next = voxel + Eigen::Vector3i(dx, dy, dz);
          if ((dx == 0 && dy == 0 && dz == 0) || !geometry.contains(next)) {
            continue;
          }
          const auto enterable = [&](const Eigen::Vector3i & into) {
            const std::uint8_t into_cost = volume.cost(into);
            return into_cost < kInscribedCost || (into_cost == kInscribedCost && volume.cost(voxel) >= kInscribedCost);
          };
          // Nor past the edge or corner of a voxel it could not enter
          bool cuts_corner = false;
          for (int mask = 1; mask < 7; ++mask) {
            const Eigen::Vector3i part{mask & 1 ? dx : 0, mask & 2 ? dy : 0, mask & 4 ? dz : 0};
            cuts_corner = cuts_corner || !enterable(voxel + part);
          }
          if (!enterable(next) || cuts_corner) {
            continue;
          }
          const std::uint8_t next_cost = volume.cost(next);
          const std::uint8_t paid = volume.known(next) ? next_cost : std::max(next_cost, config.unknown_cost);
          const double length = geometry.resolution * std::hypot(dx, dy, config.z_weight * dz);
          const double next_total = cost + length * (1.0 + config.cost_weight * paid / 252.0);
          if (next_total < best[geometry.index(next)]) {
            best[geometry.index(next)] = next_total;
            queue.push({next_total, next});
          }
        }
      }
    }
  }
  return best[geometry.index(goal)];
}

}  // namespace

// --- Search ---

TEST_CASE("plan_through_voxels goes straight through open water", "[planner_math]")
{
  Tank tank({8.0, 4.0, 3.0});
  const CostVolume volume = tank.volume();
  const Eigen::Vector3d start{1.03, 2.04, -1.52};
  const Eigen::Vector3d goal{6.97, 2.51, -2.08};

  const Plan3D result = plan(volume, start, goal);

  REQUIRE(result.status == PlanStatus::kSucceeded);
  REQUIRE(result.points.size() == 2);
  REQUIRE(result.points.front() == start);
  REQUIRE(result.points.back() == goal);
  // The voxel route is a little longer than the straight line, which the heuristic aims along
  const Eigen::Vector3d offset = goal - start;
  const double weighted = std::hypot(offset.x(), offset.y(), 1.5 * offset.z());
  REQUIRE(result.search_cost >= weighted - 0.2);
  REQUIRE(result.search_cost <= weighted * 1.1);
}

TEST_CASE("plan_through_voxels goes around an obstacle that spans every depth", "[planner_math]")
{
  Tank tank({8.0, 4.0, 3.0});
  // A wall from the floor to the surface across the straight line, with a way around at its north end
  tank.add_box({3.9, -1.0, -4.0}, {4.1, 2.8, 1.0});
  const CostVolume volume = tank.volume();

  const Plan3D result = plan(volume, {1.0, 1.0, -1.5}, {7.0, 1.0, -1.5});

  REQUIRE(result.status == PlanStatus::kSucceeded);
  REQUIRE(clear_of_obstacles(volume, result.points));
  REQUIRE(crossing(result.points, 4.0).y() > 2.8 + 0.35 - kVoxel);
}

TEST_CASE("plan_through_voxels goes over an obstacle on the floor", "[planner_math]")
{
  Tank tank({8.0, 3.0, 3.0});
  tank.add_floor();
  // A block on the floor from wall to wall, 0.9 m tall like the pool's shipwreck
  tank.add_box({3.0, -1.0, -4.0}, {5.0, 4.0, -2.1});
  const CostVolume volume = tank.volume();

  const Plan3D result = plan(volume, {1.0, 1.5, -2.5}, {7.0, 1.5, -2.5});

  REQUIRE(result.status == PlanStatus::kSucceeded);
  REQUIRE(clear_of_obstacles(volume, result.points));
  REQUIRE(crossing(result.points, 4.0).z() > -2.1 + 0.2 - kVoxel);
}

TEST_CASE("plan_through_voxels goes under an obstacle hanging from the surface", "[planner_math]")
{
  Tank tank({8.0, 3.0, 3.0});
  tank.add_floor();
  // A curtain from the surface down to 1.5 m deep, from wall to wall
  tank.add_box({3.9, -1.0, -1.5}, {4.1, 4.0, 1.0});
  const CostVolume volume = tank.volume();

  const Plan3D result = plan(volume, {1.0, 1.5, -1.0}, {7.0, 1.5, -1.0});

  REQUIRE(result.status == PlanStatus::kSucceeded);
  REQUIRE(clear_of_obstacles(volume, result.points));
  REQUIRE(lowest(result.points) < -1.5 - 0.2 + kVoxel);
  REQUIRE(crossing(result.points, 4.0).z() < -1.5 - 0.2 + kVoxel);
}

TEST_CASE("plan_through_voxels goes through a gate in a wall", "[planner_math]")
{
  Tank tank({8.0, 4.0, 4.0});
  tank.add_floor();
  // A wall from the floor to the surface with a 1.4 m square opening, from y = 1.3 to 2.7 and z = -3 to -1.6
  tank.add_box({3.9, -1.0, -5.0}, {4.1, 1.3, 1.0});
  tank.add_box({3.9, 2.7, -5.0}, {4.1, 5.0, 1.0});
  tank.add_box({3.9, 1.3, -5.0}, {4.1, 2.7, -3.0});
  tank.add_box({3.9, 1.3, -1.6}, {4.1, 2.7, 1.0});
  const CostVolume volume = tank.volume();

  const Plan3D result = plan(volume, {1.0, 0.8, -1.0}, {7.0, 3.2, -1.0});

  REQUIRE(result.status == PlanStatus::kSucceeded);
  REQUIRE(clear_of_obstacles(volume, result.points));
  const Eigen::Vector3d through = crossing(result.points, 4.0);
  REQUIRE(through.y() > 1.3 + 0.35 - kVoxel);
  REQUIRE(through.y() < 2.7 - 0.35 + kVoxel);
  REQUIRE(through.z() > -3.0 + 0.2 - kVoxel);
  REQUIRE(through.z() < -1.6 - 0.2 + kVoxel);
}

TEST_CASE("plan_through_voxels prefers water the sensor has seen", "[planner_math]")
{
  Tank tank({8.0, 4.0, 3.0});
  // Only a lane at y = 0.5 to 1.0 has been seen
  const auto in_lane = [](const Eigen::Vector3d & point) { return point.y() >= 0.5 && point.y() <= 1.0; };
  const CostVolume volume = tank.volume(in_lane);
  const Eigen::Vector3d start{1.0, 2.0, -1.5};
  const Eigen::Vector3d goal{7.0, 2.0, -1.5};

  SECTION("Unknown water costs more, so the route detours through the lane")
  {
    PlannerConfig config;
    config.unknown_cost = 252;
    const Plan3D result = plan_through_voxels(volume, start, goal, config, never_canceled);
    REQUIRE(result.status == PlanStatus::kSucceeded);
    REQUIRE(in_lane(crossing(result.points, 4.0)));
  }

  SECTION("Unknown water at no extra cost is crossed straight")
  {
    PlannerConfig config;
    config.unknown_cost = 0;
    const Plan3D result = plan_through_voxels(volume, start, goal, config, never_canceled);
    REQUIRE(result.status == PlanStatus::kSucceeded);
    REQUIRE(result.points.size() == 2);
  }
}

TEST_CASE("plan_through_voxels gets a vehicle that is touching the floor away from it", "[planner_math]")
{
  Tank tank({8.0, 3.0, 3.0});
  tank.add_floor();
  const CostVolume volume = tank.volume();
  // 0.1 m above the floor, within the vehicle's 0.2 m half-height of it
  const Eigen::Vector3d start{1.0, 1.5, -2.8};
  REQUIRE(volume.cost(*volume.geometry().voxel_at(start)) == kInscribedCost);

  const Plan3D result = plan(volume, start, {7.0, 1.5, -2.0});

  REQUIRE(result.status == PlanStatus::kSucceeded);
  REQUIRE(result.points.front() == start);
  REQUIRE(result.points.back() == Eigen::Vector3d(7.0, 1.5, -2.0));
}

TEST_CASE("plan_through_voxels reports why it cannot plan", "[planner_math]")
{
  Tank tank({8.0, 3.0, 3.0});
  tank.add_floor();
  // A solid block around (6, 1.5, -1.5)
  tank.add_box({5.0, 0.5, -2.5}, {7.0, 2.5, -0.5});
  const CostVolume solid = tank.volume();
  const Eigen::Vector3d start{1.0, 1.5, -1.5};

  SECTION("Start or goal outside the grid")
  {
    REQUIRE(plan(solid, {-1.0, 1.5, -1.5}, {2.0, 1.5, -1.5}).status == PlanStatus::kStartOutside);
    REQUIRE(plan(solid, start, {2.0, 1.5, 0.5}).status == PlanStatus::kGoalOutside);
  }

  SECTION("Start in an obstacle, or goal within the vehicle's size of one")
  {
    REQUIRE(plan(solid, {6.0, 1.5, -1.5}, start).status == PlanStatus::kStartOccupied);
    REQUIRE(plan(solid, start, {6.0, 1.5, -1.5}).status == PlanStatus::kGoalOccupied);
    REQUIRE(plan(solid, start, {4.8, 1.5, -1.5}).status == PlanStatus::kGoalOccupied);
  }

  SECTION("Obstacles block every route")
  {
    // A hollow shell around (6, 1.5, -1.5), with room inside for the vehicle
    Tank shell({8.0, 3.0, 3.0});
    shell.add_box({4.6, 0.1, -2.9}, {7.4, 0.2, -0.1});
    shell.add_box({4.6, 2.8, -2.9}, {7.4, 2.9, -0.1});
    shell.add_box({4.6, 0.1, -2.9}, {4.7, 2.9, -0.1});
    shell.add_box({7.3, 0.1, -2.9}, {7.4, 2.9, -0.1});
    shell.add_box({4.6, 0.1, -2.9}, {7.4, 2.9, -2.8});
    shell.add_box({4.6, 0.1, -0.2}, {7.4, 2.9, -0.1});
    const CostVolume volume = shell.volume();
    REQUIRE(volume.cost(*volume.geometry().voxel_at({6.0, 1.5, -1.5})) < kInscribedCost);

    const Plan3D result = plan(volume, start, {6.0, 1.5, -1.5});
    REQUIRE(result.status == PlanStatus::kNoPath);
    REQUIRE(result.points.empty());

    SECTION("Canceled, or out of time, before it finds that out")
    {
      PlannerConfig config;
      REQUIRE(plan_through_voxels(volume, start, {6.0, 1.5, -1.5}, config, [] {
                return true;
              }).status == PlanStatus::kCanceled);
      config.max_planning_time = 0.0;
      REQUIRE(
        plan_through_voxels(volume, start, {6.0, 1.5, -1.5}, config, never_canceled).status == PlanStatus::kTimedOut);
    }
  }
}

TEST_CASE("plan_through_voxels finds the cheapest route through the grid", "[planner_math]")
{
  GridGeometry geometry;
  geometry.resolution = 0.1;
  geometry.size = {9, 7, 5};
  std::mt19937 random(7);
  const std::vector<std::uint8_t> free_levels{0, 0, 30, 120, 200, 252};
  std::uniform_int_distribution<std::size_t> free_level(0, free_levels.size() - 1);
  std::bernoulli_distribution lethal(0.5);
  std::bernoulli_distribution seen(0.6);
  PlannerConfig config;
  config.unknown_cost = 90;

  int solved = 0;
  for (int trial = 0; trial < 40; ++trial) {
    // From a few blocked voxels to most of them, so some trials have no route
    std::bernoulli_distribution blocked(0.1 + 0.6 * trial / 40.0);
    std::vector<std::uint8_t> costs(geometry.count());
    std::vector<bool> known(geometry.count());
    for (std::size_t n = 0; n < costs.size(); ++n) {
      costs[n] = blocked(random) ? (lethal(random) ? kLethalCost : kInscribedCost) : free_levels[free_level(random)];
      known[n] = seen(random);
    }
    const Eigen::Vector3i start{0, 0, 0};
    const Eigen::Vector3i goal{8, 6, 4};
    costs[geometry.index(start)] = kFreeCost;
    costs[geometry.index(goal)] = kFreeCost;
    const CostVolume volume(geometry, costs, known);

    const Plan3D result =
      plan_through_voxels(volume, geometry.center(start), geometry.center(goal), config, never_canceled);
    const double expected = cheapest_cost(volume, start, goal, config);

    if (std::isinf(expected)) {
      REQUIRE(result.status == PlanStatus::kNoPath);
    } else {
      REQUIRE(result.status == PlanStatus::kSucceeded);
      REQUIRE_THAT(result.search_cost, WithinAbs(expected, 1e-9));
      REQUIRE(clear_of_obstacles(volume, result.points));
      ++solved;
    }
  }
  // Both outcomes are exercised
  REQUIRE(solved > 5);
  REQUIRE(solved < 40);
}

// --- Heading ---

TEST_CASE("add_yaw turns evenly from the start's heading to the goal's", "[planner_math]")
{
  const std::vector<Eigen::Vector3d> points{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 3.0, 0.0}};

  const auto path = add_yaw(points, 3.0, -3.0, YawMode::kInterpolate);

  REQUIRE(path.size() == 3);
  REQUIRE(path.front().yaw == 3.0);
  REQUIRE(path.back().yaw == -3.0);
  // A quarter of the way along, a quarter of the short turn through +/-pi
  const double turn = angles::shortest_angular_distance(3.0, -3.0);
  REQUIRE_THAT(angles::shortest_angular_distance(3.0 + turn / 4.0, path[1].yaw), WithinAbs(0.0, kTolerance));
}

TEST_CASE("add_yaw turns in place when the route does not move", "[planner_math]")
{
  const std::vector<Eigen::Vector3d> points{{1.0, 2.0, -1.0}, {1.0, 2.0, -1.0}};

  for (const YawMode mode : {YawMode::kInterpolate, YawMode::kFaceTravel}) {
    const auto path = add_yaw(points, 0.0, 1.5, mode);
    REQUIRE(path.front().yaw == 0.0);
    REQUIRE(path.back().yaw == 1.5);
    REQUIRE(path.back().position == points.back());
  }
}

TEST_CASE("add_yaw with kFaceTravel faces along the route", "[planner_math]")
{
  // East, then a steep climb, then north
  const std::vector<Eigen::Vector3d> points{{0.0, 0.0, -2.0}, {2.0, 0.0, -2.0}, {2.1, 0.0, -1.0}, {2.1, 2.0, -1.0}};

  const auto path = add_yaw(points, 1.0, -1.0, YawMode::kFaceTravel);

  const std::vector<Pose4D> expected{
    {points[0], 1.0},  // the start
    {points[0], 0.0},  // turn east in place
    {points[1], 0.0},
    {points[2], 0.0},  // the climb keeps the heading
    {points[2], M_PI / 2.0},  // turn north in place
    {points[3], M_PI / 2.0},
    {points[3], -1.0},  // turn to the goal's heading in place
  };
  REQUIRE(path.size() == expected.size());
  for (std::size_t i = 0; i < path.size(); ++i) {
    REQUIRE((path[i].position - expected[i].position).norm() < kTolerance);
    REQUIRE_THAT(path[i].yaw, WithinAbs(expected[i].yaw, kTolerance));
  }
}

TEST_CASE("add_yaw needs a route", "[planner_math]")
{
  REQUIRE_THROWS_AS(add_yaw({{0.0, 0.0, 0.0}}, 0.0, 0.0, YawMode::kInterpolate), std::invalid_argument);
}

// --- With the controller ---

TEST_CASE("A route planned over a block keeps the hull clear when pure pursuit flies it", "[planner_math][closed_loop]")
{
  Tank tank({10.0, 3.0, 3.0});
  tank.add_floor();
  // A block from wall to wall, 0.9 m tall, so the only way is over it
  const Eigen::Vector3d block_low{4.0, -1.0, -4.0};
  const Eigen::Vector3d block_high{6.0, 4.0, -2.1};
  tank.add_box(block_low, block_high);
  const CostVolume volume = tank.volume();

  const Eigen::Vector3d start{1.0, 1.5, -2.5};
  const Eigen::Vector3d goal{9.0, 1.5, -2.5};
  const Plan3D result = plan(volume, start, goal);
  REQUIRE(result.status == PlanStatus::kSucceeded);
  const auto path = add_yaw(result.points, 0.0, 0.0, YawMode::kInterpolate);

  // The controller in action_bringup's nav2.yaml, flying a vehicle that follows its commands through a lag
  PursuitConfig config;
  config.lookahead_dist = 0.6;
  config.approach_gain = 1.5;
  config.max_vel.linear = Eigen::Vector3d(0.5, 0.3, 0.3);
  config.max_vel.yaw = 0.6;
  config.max_accel.linear = Eigen::Vector3d(0.4, 0.3, 0.3);
  config.max_accel.yaw = 0.6;
  const Eigen::Vector3d hull{0.23, 0.29, 0.127};  // m, half of the ROV's length, width and height

  for (const double response_time : {0.0, 0.5}) {
    Pursuit pursuit(config);
    pursuit.set_path(path);
    Pose4D pose{start, 0.0};
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    double clearance = std::numeric_limits<double>::infinity();
    constexpr double kDt = 0.05;
    for (double t = 0.0; t < 60.0; t += kDt) {
      const BodyVelocity command = pursuit.compute(pose, kDt);
      const double blend = response_time > 0.0 ? 1.0 - std::exp(-kDt / response_time) : 1.0;
      velocity += blend * (command.linear - velocity);
      pose.position += Eigen::AngleAxisd(pose.yaw, Eigen::Vector3d::UnitZ()) * velocity * kDt;
      pose.yaw += command.yaw_rate * kDt;

      // The gap between the hull and the block, and the hull and the floor's top at z = -2.9
      const Eigen::Vector3d gaps = (block_low - (pose.position + hull)).cwiseMax((pose.position - hull) - block_high);
      const double block_gap = (gaps.array() > 0.0).any() ? gaps.cwiseMax(0.0).norm() : gaps.maxCoeff();
      clearance = std::min({clearance, block_gap, pose.position.z() - hull.z() - (-2.9)});
    }
    INFO("response time " << response_time << " s");
    REQUIRE((pose.position - goal).norm() < 0.05);
    // Measured: 0.27 m following commands exactly, 0.20 m with the lag
    REQUIRE(clearance > 0.15);
  }
}
