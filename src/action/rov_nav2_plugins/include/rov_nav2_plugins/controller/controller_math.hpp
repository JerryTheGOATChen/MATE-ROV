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
#include <Eigen/Core>
#include <optional>
#include <vector>

#include "rov_nav2_plugins/common/pose4d.hpp"

// PurePursuitController3D's navigation math, independent of Nav2 and ROS communication.

namespace rov_nav2_plugins
{
// --- Speed profile and limits ---

/**
 * @brief Speed for closing `distance` to a target: as fast as allowed while still able to stop on it.
 *
 * Returns the smallest of:
 *  - `max_speed`,
 *  - the speed from which decelerating at `max_accel` stops exactly at the target, and
 *  - `approach_gain * distance`, a proportional final approach. The braking curve alone has
 *    unbounded gain at zero distance, which makes a vehicle with any response lag chatter
 *    around the target.
 */
double approach_speed(double distance, double max_speed, double max_accel, double approach_gain);

/// Move `current` toward `target` by at most `max_step`.
double step_toward(double current, double target, double max_step);

/**
 * @brief Largest scale `s` such that `s * |direction[i]| <= limits[i]` on every axis.
 *
 * Scaling the whole vector, rather than clipping axes independently, keeps its direction, so the
 * vehicle still heads straight at its target when one axis saturates. Axes where `direction` is
 * zero impose no limit.
 */
double max_scale_within(const Eigen::Vector3d & direction, const Eigen::Vector3d & limits);

// --- Progress along the path ---

/**
 * @brief Tracks a vehicle's progress along a 3D polyline path.
 *
 * The path is parameterized by arc length. Between path poses, position is interpolated linearly
 * and yaw along the shortest arc. Progress only moves forward, so a path that crosses itself
 * (e.g. a survey loop) is followed in order.
 */
class PathTracker
{
public:
  /// @throws std::invalid_argument if `poses` is empty
  explicit PathTracker(std::vector<Pose4D> poses);

  /**
   * @brief Advance progress to the path point closest to `position`.
   *
   * Only the stretch of path up to `search_dist` (arc length) ahead of the current progress is
   * searched, so progress cannot jump to a later pass of a path that crosses itself.
   */
  void update_progress(const Eigen::Vector3d & position, double search_dist);

  /// Pose at arc length `s`, clamped to the path. At or past the end, this is exactly the last pose.
  Pose4D pose_at(double s) const;

  /// Arc length of the vehicle's progress along the path.
  double progress() const;

  /// Total arc length of the path.
  double length() const;

private:
  /// Index of the segment (from pose i to pose i + 1) that contains arc length `s`.
  std::size_t segment_containing(double s) const;

  std::vector<Pose4D> poses_;
  std::vector<double> arc_lengths_;  // arc length at each pose
  double progress_{0.0};
};

// --- The control law ---

/// Velocity in the vehicle (base_link) frame: surge, sway and heave, plus yaw rate.
struct BodyVelocity
{
  Eigen::Vector3d linear{Eigen::Vector3d::Zero()};
  double yaw_rate{0.0};
};

/// Magnitude limits per vehicle axis: x (surge), y (sway), z (heave) and yaw.
struct AxisLimits
{
  Eigen::Vector3d linear{Eigen::Vector3d::Zero()};
  double yaw{0.0};
};

struct PursuitConfig
{
  double lookahead_dist{0.0};  // m of path ahead of the vehicle's progress
  double approach_gain{0.0};  // 1/s, see approach_speed()
  AxisLimits max_vel;  // m/s, rad/s
  AxisLimits max_accel;  // m/s^2, rad/s^2
};

/**
 * @brief Pure pursuit path following in 3D for a holonomic vehicle (x, y, z, yaw).
 *
 * Each step the vehicle heads straight for the lookahead point, `lookahead_dist` along the path
 * ahead of its progress, and turns toward the path's yaw at that point. Translation and rotation
 * each follow a trapezoidal speed profile (see approach_speed()) that brakes to a stop at the end
 * of the path. Velocity and acceleration are limited per body axis, relative to the previous
 * command for acceleration, and scaled as a whole so the direction is kept when an axis saturates.
 */
class Pursuit
{
public:
  /// @throws std::invalid_argument if any limit or the lookahead is not positive
  explicit Pursuit(const PursuitConfig & config);

  /// Follow a new path. The previous command is kept so a replanned path does not jerk the vehicle.
  void set_path(std::vector<Pose4D> path);

  bool has_path() const;

  /// Forget the path and the previous command, e.g. when a navigation task ends.
  void clear();

  /**
   * @brief Velocity command for a vehicle at `pose`, `dt` seconds after the previous command.
   * @throws std::logic_error if no path has been set
   */
  BodyVelocity compute(const Pose4D & pose, double dt);

private:
  /// Unconstrained-acceleration command toward `target`, `distance_to_go` from the end of the path.
  BodyVelocity desired_velocity(const Pose4D & pose, const Pose4D & target, double distance_to_go) const;

  PursuitConfig config_;
  std::optional<PathTracker> tracker_;
  BodyVelocity last_command_;
};

}  // namespace rov_nav2_plugins
