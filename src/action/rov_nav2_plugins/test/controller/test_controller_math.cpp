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
#include <Eigen/Geometry>
#include <limits>
#include <stdexcept>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "angles/angles.h"
#include "rov_nav2_plugins/controller/controller_math.hpp"

using Catch::Matchers::WithinAbs;
using rov_nav2_plugins::approach_speed;
using rov_nav2_plugins::BodyVelocity;
using rov_nav2_plugins::max_scale_within;
using rov_nav2_plugins::PathTracker;
using rov_nav2_plugins::Pose4D;
using rov_nav2_plugins::Pursuit;
using rov_nav2_plugins::PursuitConfig;
using rov_nav2_plugins::step_toward;

namespace
{

constexpr double kTolerance = 1e-9;

// Chosen so each regime of the profile is reachable: the braking curve sqrt(2 * 0.5 * d) = sqrt(d)
// meets the speed limit at d = 1 and the proportional approach 4 * d at d = 1/16.
constexpr double kMaxSpeed = 1.0;
constexpr double kMaxAccel = 0.5;
constexpr double kGain = 4.0;

Pose4D pose(double x, double y, double z, double yaw = 0.0)
{
  return {{x, y, z}, yaw};
}

constexpr double kDt = 0.05;  // s, a 20 Hz control loop

// With a time step this long, acceleration limits cannot bind, so compute() returns the
// desired velocity and the steering law can be checked directly.
constexpr double kUnlimitedAccelDt = 100.0;

PursuitConfig test_config()
{
  PursuitConfig config;
  config.lookahead_dist = 0.6;
  config.approach_gain = 1.5;
  config.max_vel.linear = Eigen::Vector3d(0.5, 0.3, 0.3);
  config.max_vel.yaw = 0.6;
  config.max_accel.linear = Eigen::Vector3d(0.4, 0.3, 0.3);
  config.max_accel.yaw = 0.6;
  return config;
}

/// The tracker interpolates between poses, so a straight path needs only its ends.
std::vector<Pose4D> straight_path(const Pose4D & start, const Pose4D & goal)
{
  return {start, goal};
}

/// Straight legs through `waypoints`, joined the way Nav2's planner server joins them.
std::vector<Pose4D> path_through(const std::vector<Pose4D> & waypoints)
{
  std::vector<Pose4D> path{waypoints.front()};
  for (std::size_t i = 1; i < waypoints.size(); ++i) {
    const auto leg = straight_path(waypoints[i - 1], waypoints[i]);
    path.insert(path.end(), leg.begin() + 1, leg.end());
  }
  return path;
}

double distance_to_path(const std::vector<Pose4D> & path, const Eigen::Vector3d & point)
{
  double closest = (path.front().position - point).norm();
  for (std::size_t i = 1; i < path.size(); ++i) {
    const Eigen::Vector3d & start = path[i - 1].position;
    const Eigen::Vector3d segment = path[i].position - start;
    const double t =
      segment.squaredNorm() > 0.0 ? std::clamp((point - start).dot(segment) / segment.squaredNorm(), 0.0, 1.0) : 0.0;
    closest = std::min(closest, (start + t * segment - point).norm());
  }
  return closest;
}

double position_error(const Pose4D & a, const Pose4D & b)
{
  return (a.position - b.position).norm();
}

double yaw_error(const Pose4D & a, const Pose4D & b)
{
  return std::abs(angles::shortest_angular_distance(a.yaw, b.yaw));
}

/// Vehicle whose velocity follows commands through a first-order lag while a current pushes it.
struct TestVehicle
{
  explicit TestVehicle(const Pose4D & start)
  : pose(start)
  {}

  Pose4D pose;
  double response_time{0.0};  // s; zero follows commands exactly
  Eigen::Vector3d current{Eigen::Vector3d::Zero()};  // m/s, world frame
  BodyVelocity velocity;

  void step(const BodyVelocity & command, double dt)
  {
    const double blend = response_time > 0.0 ? 1.0 - std::exp(-dt / response_time) : 1.0;
    velocity.linear += blend * (command.linear - velocity.linear);
    velocity.yaw_rate += blend * (command.yaw_rate - velocity.yaw_rate);

    const Eigen::Vector3d world_velocity =
      Eigen::AngleAxisd(pose.yaw, Eigen::Vector3d::UnitZ()) * velocity.linear + current;
    pose.position += world_velocity * dt;
    pose.yaw = angles::normalize_angle(pose.yaw + velocity.yaw_rate * dt);
  }
};

bool respects_limits(const BodyVelocity & command, const BodyVelocity & previous, const PursuitConfig & config)
{
  bool ok = std::abs(command.yaw_rate) <= config.max_vel.yaw + kTolerance &&
            std::abs(command.yaw_rate - previous.yaw_rate) <= config.max_accel.yaw * kDt + kTolerance;
  for (Eigen::Index i = 0; i < 3; ++i) {
    ok = ok && std::abs(command.linear[i]) <= config.max_vel.linear[i] + kTolerance &&
         std::abs(command.linear[i] - previous.linear[i]) <= config.max_accel.linear[i] * kDt + kTolerance;
  }
  return ok;
}

struct RunResult
{
  Pose4D final_pose;
  double arrival_time{std::numeric_limits<double>::infinity()};  // first time within 5 cm and 0.05 rad of the goal
  double max_path_deviation{0.0};
  bool within_limits{true};  // every command respected the velocity and acceleration limits
};

/// Follow `path` in closed loop with `vehicle` for `duration` seconds.
RunResult run_closed_loop(const std::vector<Pose4D> & path, TestVehicle vehicle, double duration)
{
  const PursuitConfig config = test_config();
  Pursuit pursuit(config);
  pursuit.set_path(path);

  RunResult result;
  BodyVelocity previous;
  for (double t = 0.0; t < duration; t += kDt) {
    const BodyVelocity command = pursuit.compute(vehicle.pose, kDt);
    result.within_limits = result.within_limits && respects_limits(command, previous, config);
    previous = command;

    vehicle.step(command, kDt);
    result.max_path_deviation = std::max(result.max_path_deviation, distance_to_path(path, vehicle.pose.position));

    const bool arrived =
      position_error(vehicle.pose, path.back()) < 0.05 && yaw_error(vehicle.pose, path.back()) < 0.05;
    if (arrived && std::isinf(result.arrival_time)) {
      result.arrival_time = t;
    }
  }

  result.final_pose = vehicle.pose;
  return result;
}

}  // namespace

TEST_CASE("approach_speed cruises at the speed limit far from the target", "[motion_profile]")
{
  REQUIRE_THAT(approach_speed(4.0, kMaxSpeed, kMaxAccel, kGain), WithinAbs(1.0, kTolerance));
}

TEST_CASE("approach_speed brakes so the vehicle can stop on the target", "[motion_profile]")
{
  REQUIRE_THAT(approach_speed(0.25, kMaxSpeed, kMaxAccel, kGain), WithinAbs(std::sqrt(0.25), kTolerance));
}

TEST_CASE("approach_speed is proportional close to the target", "[motion_profile]")
{
  REQUIRE_THAT(approach_speed(0.01, kMaxSpeed, kMaxAccel, kGain), WithinAbs(0.04, kTolerance));
}

TEST_CASE("approach_speed is zero on or past the target", "[motion_profile]")
{
  REQUIRE(approach_speed(0.0, kMaxSpeed, kMaxAccel, kGain) == 0.0);
  REQUIRE(approach_speed(-1.0, kMaxSpeed, kMaxAccel, kGain) == 0.0);
}

TEST_CASE("step_toward moves by at most max_step", "[motion_profile]")
{
  REQUIRE_THAT(step_toward(0.0, 1.0, 0.1), WithinAbs(0.1, kTolerance));
  REQUIRE_THAT(step_toward(0.0, -1.0, 0.1), WithinAbs(-0.1, kTolerance));
  REQUIRE_THAT(step_toward(0.0, 0.05, 0.1), WithinAbs(0.05, kTolerance));
}

TEST_CASE("max_scale_within keeps every axis within its limit", "[motion_profile]")
{
  const Eigen::Vector3d limits(0.5, 0.3, 0.2);

  SECTION("Along one axis")
  {
    REQUIRE_THAT(max_scale_within({0.0, -1.0, 0.0}, limits), WithinAbs(0.3, kTolerance));
  }

  SECTION("The most constrained axis decides")
  {
    // 0.5 / 0.6 = 0.83 on x, 0.3 / 0.8 = 0.375 on y
    REQUIRE_THAT(max_scale_within({0.6, 0.8, 0.0}, limits), WithinAbs(0.375, kTolerance));
  }

  SECTION("A zero direction is unconstrained")
  {
    REQUIRE(std::isinf(max_scale_within(Eigen::Vector3d::Zero(), limits)));
  }
}

TEST_CASE("PathTracker rejects an empty path", "[path_tracker]")
{
  REQUIRE_THROWS_AS(PathTracker(std::vector<Pose4D>{}), std::invalid_argument);
}

TEST_CASE("PathTracker measures arc length in 3D", "[path_tracker]")
{
  const PathTracker tracker({pose(0, 0, 0), pose(3, 4, 0), pose(3, 4, -2)});
  REQUIRE_THAT(tracker.length(), WithinAbs(7.0, kTolerance));
}

TEST_CASE("PathTracker interpolates position and yaw", "[path_tracker]")
{
  const PathTracker tracker({pose(0, 0, 0, 0.0), pose(2, 0, -2, M_PI_2)});
  const double length = std::sqrt(8.0);

  SECTION("Between poses")
  {
    const Pose4D mid = tracker.pose_at(length / 2.0);
    REQUIRE_THAT(mid.position.x(), WithinAbs(1.0, kTolerance));
    REQUIRE_THAT(mid.position.y(), WithinAbs(0.0, kTolerance));
    REQUIRE_THAT(mid.position.z(), WithinAbs(-1.0, kTolerance));
    REQUIRE_THAT(mid.yaw, WithinAbs(M_PI_4, kTolerance));
  }

  SECTION("Clamped to the ends of the path")
  {
    REQUIRE_THAT(tracker.pose_at(-1.0).position.norm(), WithinAbs(0.0, kTolerance));
    REQUIRE_THAT(tracker.pose_at(length + 1.0).position.x(), WithinAbs(2.0, kTolerance));
    REQUIRE_THAT(tracker.pose_at(length + 1.0).yaw, WithinAbs(M_PI_2, kTolerance));
  }
}

TEST_CASE("PathTracker turns the short way across +/-pi", "[path_tracker]")
{
  // From 3.0 rad to -3.0 rad, the short way passes through pi rather than 0
  const PathTracker tracker({pose(0, 0, 0, 3.0), pose(1, 0, 0, -3.0)});
  REQUIRE_THAT(std::abs(tracker.pose_at(0.5).yaw), WithinAbs(M_PI, 1e-6));
}

TEST_CASE("PathTracker ends with exactly the last pose", "[path_tracker]")
{
  // The final two poses share a position: a turn in place at the end of the path
  const PathTracker tracker({pose(0, 0, 0, 0.0), pose(1, 0, 0, 0.0), pose(1, 0, 0, 1.0)});

  REQUIRE_THAT(tracker.length(), WithinAbs(1.0, kTolerance));
  REQUIRE_THAT(tracker.pose_at(0.5).yaw, WithinAbs(0.0, kTolerance));
  REQUIRE_THAT(tracker.pose_at(1.0).yaw, WithinAbs(1.0, kTolerance));
}

TEST_CASE("PathTracker accepts a single-pose path", "[path_tracker]")
{
  PathTracker tracker({pose(1, 2, 3, 0.5)});
  tracker.update_progress({0, 0, 0}, 1.0);

  REQUIRE(tracker.length() == 0.0);
  REQUIRE(tracker.progress() == 0.0);
  REQUIRE_THAT(tracker.pose_at(10.0).yaw, WithinAbs(0.5, kTolerance));
}

TEST_CASE("PathTracker progress follows the closest path point", "[path_tracker]")
{
  PathTracker tracker({pose(0, 0, 0), pose(10, 0, 0)});

  SECTION("An off-path position projects onto the path")
  {
    tracker.update_progress({0.5, 0.3, -0.2}, 1.0);
    REQUIRE_THAT(tracker.progress(), WithinAbs(0.5, kTolerance));
  }

  SECTION("Progress advances at most search_dist per update")
  {
    tracker.update_progress({5.0, 0, 0}, 1.0);
    REQUIRE_THAT(tracker.progress(), WithinAbs(1.0, kTolerance));
  }

  SECTION("Progress never moves backwards")
  {
    tracker.update_progress({0.8, 0, 0}, 1.0);
    tracker.update_progress({0.2, 0, 0}, 1.0);
    REQUIRE_THAT(tracker.progress(), WithinAbs(0.8, kTolerance));
  }
}

TEST_CASE("PathTracker follows a self-crossing path in order", "[path_tracker]")
{
  // The last leg crosses the first at (1, 0): arc length 1 on the first pass, 5 on the second
  PathTracker tracker({pose(0, 0, 0), pose(2, 0, 0), pose(2, 1, 0), pose(1, 1, 0), pose(1, -1, 0)});

  tracker.update_progress({1.0, 0, 0}, 1.5);
  REQUIRE_THAT(tracker.progress(), WithinAbs(1.0, kTolerance));

  for (double s = 1.0; s <= 5.0; s += 0.25) {
    tracker.update_progress(tracker.pose_at(s).position, 1.5);
  }
  REQUIRE_THAT(tracker.progress(), WithinAbs(5.0, kTolerance));
}

TEST_CASE("Pursuit validates its configuration", "[pursuit]")
{
  PursuitConfig config = test_config();

  SECTION("Lookahead")
  {
    config.lookahead_dist = 0.0;
    REQUIRE_THROWS_AS(Pursuit(config), std::invalid_argument);
  }

  SECTION("Velocity limits")
  {
    config.max_vel.linear.z() = 0.0;
    REQUIRE_THROWS_AS(Pursuit(config), std::invalid_argument);
  }

  SECTION("Acceleration limits")
  {
    config.max_accel.yaw = -1.0;
    REQUIRE_THROWS_AS(Pursuit(config), std::invalid_argument);
  }
}

TEST_CASE("Pursuit needs a path before it can compute a command", "[pursuit]")
{
  Pursuit pursuit(test_config());
  REQUIRE_FALSE(pursuit.has_path());
  REQUIRE_THROWS_AS(pursuit.compute(Pose4D{}, kDt), std::logic_error);
}

TEST_CASE("Pursuit steers toward the path in the vehicle frame", "[pursuit]")
{
  Pursuit pursuit(test_config());
  pursuit.set_path(straight_path({{0, 0, 0}, 0.0}, {{10, 0, 0}, 0.0}));

  // Facing +y, so the path ahead along +x is to the vehicle's right (-y in its frame)
  const BodyVelocity command = pursuit.compute({{0, 0, 0}, M_PI_2}, kUnlimitedAccelDt);

  REQUIRE_THAT(command.linear.x(), WithinAbs(0.0, kTolerance));
  REQUIRE_THAT(command.linear.y(), WithinAbs(-0.3, kTolerance));  // at the sway limit
  REQUIRE_THAT(command.linear.z(), WithinAbs(0.0, kTolerance));
  REQUIRE_THAT(command.yaw_rate, WithinAbs(-0.6, kTolerance));  // turning back to the path's heading
}

TEST_CASE("Pursuit keeps its direction of travel when one axis saturates", "[pursuit]")
{
  Pursuit pursuit(test_config());
  pursuit.set_path(straight_path({{0, 0, 0}, 0.0}, {{3, 0, -4}, 0.0}));

  const BodyVelocity command = pursuit.compute({{0, 0, 0}, 0.0}, kUnlimitedAccelDt);

  // Heading (0.6, 0, -0.8): heave saturates at 0.3 m/s, so surge is scaled to 0.3 * 0.6 / 0.8
  REQUIRE_THAT(command.linear.z(), WithinAbs(-0.3, kTolerance));
  REQUIRE_THAT(command.linear.x(), WithinAbs(0.225, kTolerance));
}

TEST_CASE("Pursuit limits acceleration without changing direction", "[pursuit]")
{
  Pursuit pursuit(test_config());
  pursuit.set_path(straight_path({{0, 0, 0}, 0.0}, {{3, 0, -4}, 1.0}));

  const BodyVelocity command = pursuit.compute({{0, 0, 0}, 0.0}, kDt);

  // From rest toward (0.6, 0, -0.8): heave's limit binds first, and surge keeps the same ratio
  REQUIRE_THAT(command.linear.z(), WithinAbs(-0.3 * kDt, kTolerance));
  REQUIRE_THAT(command.linear.x(), WithinAbs(0.3 * kDt * 0.6 / 0.8, kTolerance));
  // Yaw is limited on its own
  REQUIRE_THAT(command.yaw_rate, WithinAbs(0.6 * kDt, kTolerance));
}

TEST_CASE("Pursuit keeps the previous command across a new path", "[pursuit]")
{
  Pursuit pursuit(test_config());
  pursuit.set_path(straight_path({{0, 0, 0}, 0.0}, {{10, 0, 0}, 0.0}));
  BodyVelocity cruising;
  for (int i = 0; i < 40; ++i) {
    cruising = pursuit.compute({{0, 0, 0}, 0.0}, kDt);
  }
  REQUIRE_THAT(cruising.linear.x(), WithinAbs(0.5, kTolerance));

  // A replanned path in the opposite direction must not jerk the vehicle to a stop
  pursuit.set_path(straight_path({{0, 0, 0}, 0.0}, {{-10, 0, 0}, 0.0}));
  const BodyVelocity next = pursuit.compute({{0, 0, 0}, 0.0}, kDt);
  REQUIRE_THAT(next.linear.x(), WithinAbs(0.5 - 0.4 * kDt, kTolerance));
}

TEST_CASE("Pursuit clear() forgets the path and the previous command", "[pursuit]")
{
  Pursuit pursuit(test_config());
  const auto path = straight_path({{0, 0, 0}, 0.0}, {{10, 0, 0}, 0.0});
  pursuit.set_path(path);
  for (int i = 0; i < 40; ++i) {
    pursuit.compute({{0, 0, 0}, 0.0}, kDt);
  }

  pursuit.clear();
  REQUIRE_FALSE(pursuit.has_path());

  // Starts again from rest
  pursuit.set_path(path);
  REQUIRE_THAT(pursuit.compute({{0, 0, 0}, 0.0}, kDt).linear.x(), WithinAbs(0.4 * kDt, kTolerance));
}

TEST_CASE("Pursuit drives a vehicle to a 3D goal and stops there", "[pursuit][closed_loop]")
{
  const auto path = straight_path({{0, 0, 0}, 0.0}, {{4, 2, -1.5}, M_PI_2});
  const RunResult result = run_closed_loop(path, TestVehicle(path.front()), 60.0);

  REQUIRE(result.within_limits);
  REQUIRE(result.arrival_time < 15.0);
  REQUIRE(position_error(result.final_pose, path.back()) < 0.01);
  REQUIRE(yaw_error(result.final_pose, path.back()) < 0.01);
  REQUIRE(result.max_path_deviation < 0.02);
}

TEST_CASE("Pursuit dives without drifting horizontally", "[pursuit][closed_loop]")
{
  const auto path = straight_path({{1, 1, 0}, 0.5}, {{1, 1, -2}, 0.5});
  const RunResult result = run_closed_loop(path, TestVehicle(path.front()), 30.0);

  REQUIRE(result.within_limits);
  REQUIRE(result.arrival_time < 10.0);
  REQUIRE(position_error(result.final_pose, path.back()) < 0.01);
  REQUIRE(result.max_path_deviation < 1e-6);
}

TEST_CASE("Pursuit turns in place", "[pursuit][closed_loop]")
{
  const auto path = straight_path({{0, 0, -1}, 0.0}, {{0, 0, -1}, 3.0});
  const RunResult result = run_closed_loop(path, TestVehicle(path.front()), 30.0);

  REQUIRE(result.within_limits);
  REQUIRE(result.arrival_time < 8.0);
  REQUIRE(yaw_error(result.final_pose, path.back()) < 0.01);
  REQUIRE(result.max_path_deviation < 1e-6);
}

TEST_CASE("Pursuit flies a survey pattern through waypoints", "[pursuit][closed_loop]")
{
  // A 3 m square at 1 m depth, facing along each leg
  const auto path = path_through(
    {{{0, 0, -1}, 0.0}, {{3, 0, -1}, 0.0}, {{3, 3, -1}, M_PI_2}, {{0, 3, -1}, M_PI}, {{0, 0, -1}, -M_PI_2}});
  const RunResult result = run_closed_loop(path, TestVehicle(path.front()), 90.0);

  REQUIRE(result.within_limits);
  REQUIRE(result.arrival_time < 35.0);
  REQUIRE(position_error(result.final_pose, path.back()) < 0.01);
  // Pure pursuit cuts corners by a fraction of the lookahead distance
  REQUIRE(result.max_path_deviation < 0.25);
}

TEST_CASE("Pursuit reaches the goal despite response lag and a water current", "[pursuit][closed_loop]")
{
  const auto path = straight_path({{0, 0, 0}, 0.0}, {{4, 2, -1.5}, M_PI_2});
  TestVehicle vehicle(path.front());
  vehicle.response_time = 0.3;
  vehicle.current = Eigen::Vector3d(0.05, -0.03, 0.02);

  const RunResult result = run_closed_loop(path, vehicle, 60.0);

  REQUIRE(result.within_limits);
  // The current holds the vehicle off the goal by about |current| / approach_gain = 0.04 m
  REQUIRE(position_error(result.final_pose, path.back()) < 0.06);
  REQUIRE(yaw_error(result.final_pose, path.back()) < 0.01);
  REQUIRE(result.max_path_deviation < 0.15);
}
