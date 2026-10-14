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
#include "rov_nav2_plugins/controller/controller_math.hpp"

#include <algorithm>
#include <cmath>
#include <Eigen/Geometry>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <utility>

#include "angles/angles.h"

namespace rov_nav2_plugins
{

double approach_speed(double distance, double max_speed, double max_accel, double approach_gain)
{
  const double d = std::max(distance, 0.0);
  return std::min({max_speed, std::sqrt(2.0 * max_accel * d), approach_gain * d});
}

double step_toward(double current, double target, double max_step)
{
  return current + std::clamp(target - current, -max_step, max_step);
}

double max_scale_within(const Eigen::Vector3d & direction, const Eigen::Vector3d & limits)
{
  double scale = std::numeric_limits<double>::infinity();
  for (Eigen::Index i = 0; i < direction.size(); ++i) {
    const double magnitude = std::abs(direction[i]);
    if (magnitude > 0.0) {
      scale = std::min(scale, limits[i] / magnitude);
    }
  }
  return scale;
}

PathTracker::PathTracker(std::vector<Pose4D> poses)
: poses_(std::move(poses))
{
  if (poses_.empty()) {
    throw std::invalid_argument("PathTracker needs at least one pose");
  }

  arc_lengths_.reserve(poses_.size());
  arc_lengths_.push_back(0.0);
  for (std::size_t i = 1; i < poses_.size(); ++i) {
    arc_lengths_.push_back(arc_lengths_.back() + (poses_[i].position - poses_[i - 1].position).norm());
  }
}

void PathTracker::update_progress(const Eigen::Vector3d & position, double search_dist)
{
  if (poses_.size() < 2) {
    return;
  }

  const double search_end = progress_ + std::max(search_dist, 0.0);
  double closest_dist_sq = std::numeric_limits<double>::infinity();
  double closest_s = progress_;

  for (std::size_t i = segment_containing(progress_); i + 1 < poses_.size() && arc_lengths_[i] <= search_end; ++i) {
    const Eigen::Vector3d & start = poses_[i].position;
    const Eigen::Vector3d segment = poses_[i + 1].position - start;
    const double segment_length_sq = segment.squaredNorm();

    // Fraction along the segment of the point closest to `position`
    const double t =
      segment_length_sq > 0.0 ? std::clamp((position - start).dot(segment) / segment_length_sq, 0.0, 1.0) : 0.0;

    const double dist_sq = (start + t * segment - position).squaredNorm();
    if (dist_sq < closest_dist_sq) {
      closest_dist_sq = dist_sq;
      closest_s = arc_lengths_[i] + t * (arc_lengths_[i + 1] - arc_lengths_[i]);
    }
  }

  progress_ = std::clamp(closest_s, progress_, search_end);
}

Pose4D PathTracker::pose_at(double s) const
{
  s = std::max(s, 0.0);
  if (s >= length()) {
    return poses_.back();
  }

  const std::size_t i = segment_containing(s);
  const Pose4D & from = poses_[i];
  const Pose4D & to = poses_[i + 1];
  const double t = (s - arc_lengths_[i]) / (arc_lengths_[i + 1] - arc_lengths_[i]);

  return {
    from.position + t * (to.position - from.position),
    angles::normalize_angle(from.yaw + t * angles::shortest_angular_distance(from.yaw, to.yaw))};
}

double PathTracker::progress() const
{
  return progress_;
}

double PathTracker::length() const
{
  return arc_lengths_.back();
}

std::size_t PathTracker::segment_containing(double s) const
{
  // Requires s >= 0 and at least two poses. The first pose strictly beyond s has index >= 1
  // because arc_lengths_[0] == 0. Zero-length segments never contain s, so they are skipped.
  const auto next = std::upper_bound(arc_lengths_.begin(), arc_lengths_.end(), s);
  const auto next_index = static_cast<std::size_t>(std::distance(arc_lengths_.begin(), next));

  // At or past the end of the path, use the last segment
  return std::min(next_index, poses_.size() - 1) - 1;
}

namespace
{

// Closer than this, the vehicle is on the lookahead point and has no direction to head in
constexpr double kOnTargetDistance = 1e-6;  // m

bool all_positive(const AxisLimits & limits)
{
  return (limits.linear.array() > 0.0).all() && limits.yaw > 0.0;
}

}  // namespace

Pursuit::Pursuit(const PursuitConfig & config)
: config_(config)
{
  if (config_.lookahead_dist <= 0.0 || config_.approach_gain <= 0.0) {
    throw std::invalid_argument("lookahead_dist and approach_gain must be positive");
  }
  if (!all_positive(config_.max_vel) || !all_positive(config_.max_accel)) {
    throw std::invalid_argument("velocity and acceleration limits must be positive");
  }
}

void Pursuit::set_path(std::vector<Pose4D> path)
{
  tracker_.emplace(std::move(path));
}

bool Pursuit::has_path() const
{
  return tracker_.has_value();
}

void Pursuit::clear()
{
  tracker_.reset();
  last_command_ = BodyVelocity{};
}

BodyVelocity Pursuit::compute(const Pose4D & pose, double dt)
{
  if (!tracker_) {
    throw std::logic_error("Pursuit::compute() called before set_path()");
  }

  tracker_->update_progress(pose.position, config_.lookahead_dist);
  const double target_s = std::min(tracker_->progress() + config_.lookahead_dist, tracker_->length());
  const Pose4D target = tracker_->pose_at(target_s);
  const double distance_to_go = (target.position - pose.position).norm() + (tracker_->length() - target_s);

  const BodyVelocity desired = desired_velocity(pose, target, distance_to_go);

  // Limit acceleration relative to the previous command. As with speed, the change in velocity
  // keeps its direction when one axis's limit binds, so the vehicle does not veer off the path
  // while it speeds up or slows down.
  dt = std::max(dt, 0.0);
  const Eigen::Vector3d change = desired.linear - last_command_.linear;
  last_command_.linear += std::min(1.0, max_scale_within(change, config_.max_accel.linear * dt)) * change;
  last_command_.yaw_rate = step_toward(last_command_.yaw_rate, desired.yaw_rate, config_.max_accel.yaw * dt);

  return last_command_;
}

BodyVelocity Pursuit::desired_velocity(const Pose4D & pose, const Pose4D & target, double distance_to_go) const
{
  BodyVelocity desired;

  const Eigen::Vector3d offset = target.position - pose.position;
  const double offset_norm = offset.norm();
  if (offset_norm > kOnTargetDistance) {
    // Unit vector toward the target, in the vehicle frame
    const Eigen::Vector3d direction = Eigen::AngleAxisd(-pose.yaw, Eigen::Vector3d::UnitZ()) * (offset / offset_norm);

    const double max_speed = max_scale_within(direction, config_.max_vel.linear);
    const double max_accel = max_scale_within(direction, config_.max_accel.linear);
    desired.linear = approach_speed(distance_to_go, max_speed, max_accel, config_.approach_gain) * direction;
  }

  const double yaw_error = angles::shortest_angular_distance(pose.yaw, target.yaw);
  const double yaw_speed =
    approach_speed(std::abs(yaw_error), config_.max_vel.yaw, config_.max_accel.yaw, config_.approach_gain);
  desired.yaw_rate = std::copysign(yaw_speed, yaw_error);

  return desired;
}

}  // namespace rov_nav2_plugins
