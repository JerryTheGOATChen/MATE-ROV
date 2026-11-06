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

#include "ghost_brain/localizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "ghost_brain/planner.hpp"
#include "ghost_brain/tracker.hpp"

namespace ghost_brain
{

namespace
{

/// A gap between velocity messages longer than this (s) is a glitch, not real time passing: skip it
constexpr double kMaxStep = 0.5;

/// The ghost's heading (rotation about z) from its orientation quaternion, in radians
double yaw_of(const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

/// How far the ghost moved in the MAP frame, given its velocity in its OWN frame
/// (x forward, y left, z up, in m/s), its heading (yaw, radians) and the time step dt (seconds).
///   distance in its own frame = velocity * dt
///   then rotate the x, y part by the yaw into the map (the same rotation as ghost_to_map in the
///   tracker); z needs no rotation.
/// Returns the change in position, NOT the new position.
geometry_msgs::msg::Point displacement_in_map(
  const geometry_msgs::msg::Vector3 & body_velocity, double yaw, double dt)
{
  double dx = body_velocity.x * dt;
  double dy = body_velocity.y * dt;
  double dz = body_velocity.z * dt;
  geometry_msgs::msg::Point step;

  step.x = dx * cos(yaw) - dy * sin(yaw);
  step.y = dx * sin(yaw) + dy * cos(yaw);
  step.z = dz;
  return step;
}

/// How far (m) either side of the current estimate to try, and the spacing between tries
constexpr double kSearchRadius = 0.3;
constexpr double kSearchStep = 0.1;

/// A beam agrees with the map if it measured within this (m) of what the map predicts
constexpr double kInlierTolerance = 0.2;

/// Only correct if the best guess explains at least this many more beams than the current estimate
/// does, and at least kMinInliers beams in all, so noise and a bad view don't make the pose jitter
constexpr int kMinGain = 3;
constexpr int kMinInliers = 8;

/// Move this fraction of the way toward the best guess each scan (small: a scan is only a slice)
constexpr double kNudge = 0.2;

/// Height from the floor: pixels whose drop is within this (m) of the biggest one are floor, it needs
/// at least this many of them, and each image moves the height this fraction of the way
constexpr double kFloorBand = 0.05;
constexpr int kMinFloorPixels = 200;
constexpr double kHeightNudge = 0.1;
constexpr double kMaxHeightJump = 0.3;

/// How far (m) a ray from (x, y) at this angle (radians, in the MAP frame) goes before it hits
/// something blocked in the map, or infinity if it goes max_range without hitting anything.
double expected_range(
  const nav_msgs::msg::OccupancyGrid & map, double x, double y, double angle, double max_range)
{
  constexpr double kRayStep = 0.05;
  const double cos_a = std::cos(angle);
  const double sin_a = std::sin(angle);
  for (double r = kRayStep; r <= max_range; r += kRayStep) {
    if (is_occupied(map, x + r * cos_a, y + r * sin_a)) {
      return r;
    }
  }
  return std::numeric_limits<double>::infinity();
}

/// How many of the scan's beams agree with the map if the ghost is at (x, y) facing yaw. Counting,
/// not summing squared error, is on purpose: people and furniture the ghost flies over show up in
/// the sonar but not in the map, and those beams should simply not vote.
int count_agreeing_beams(
  const sensor_msgs::msg::LaserScan & scan, const nav_msgs::msg::OccupancyGrid & map, double x, double y,
  double yaw)
{
  int agreeing = 0;
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const double measured = scan.ranges[i];
    if (!std::isfinite(measured) || measured < scan.range_min || measured > scan.range_max) {
      continue;  // this beam saw nothing
    }
    // The scan's angles are counted from the ghost's nose, counterclockwise
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment + yaw;
    const double expected = expected_range(map, x, y, angle, scan.range_max);
    if (std::abs(measured - expected) < kInlierTolerance) {
      ++agreeing;
    }
  }
  return agreeing;
}

}  // namespace

void Localizer::reset(const geometry_msgs::msg::Pose & start)
{
  pose_ = start;
  speed_ = 0.0;
  has_last_velocity_ = false;
}

void Localizer::set_stationary(bool stationary)
{
  if (stationary && !stationary_) {
    bias_sum_ = geometry_msgs::msg::Vector3{};
    bias_count_ = 0;
  }
  if (!stationary && stationary_ && bias_count_ >= 10) {
    // Standing still: whatever the sensor read is its bias
    bias_.x = bias_sum_.x / bias_count_;
    bias_.y = bias_sum_.y / bias_count_;
    bias_.z = bias_sum_.z / bias_count_;
  }
  stationary_ = stationary;
}

void Localizer::on_velocity(const geometry_msgs::msg::TwistStamped & velocity_in)
{
  if (stationary_) {
    bias_sum_.x += velocity_in.twist.linear.x;
    bias_sum_.y += velocity_in.twist.linear.y;
    bias_sum_.z += velocity_in.twist.linear.z;
    ++bias_count_;
  }
  // The reading with its bias taken off
  geometry_msgs::msg::TwistStamped velocity = velocity_in;
  velocity.twist.linear.x -= bias_.x;
  velocity.twist.linear.y -= bias_.y;
  velocity.twist.linear.z -= bias_.z;
  speed_ = std::hypot(velocity.twist.linear.x, velocity.twist.linear.y, velocity.twist.linear.z);
  const rclcpp::Time now(velocity.header.stamp);
  const bool had_last = has_last_velocity_;
  const double dt = had_last ? (now - last_velocity_time_).seconds() : 0.0;
  last_velocity_time_ = now;
  has_last_velocity_ = true;
  if (stationary_ || !had_last || dt <= 0.0 || dt > kMaxStep) {
    return;  // first message of the game, or a glitch in time: nothing to integrate
  }

  // The heading comes from the compass (on_imu keeps pose_.orientation up to date)
  const double yaw = yaw_of(pose_);
  const auto step = displacement_in_map(velocity.twist.linear, yaw, dt);

  pose_.position.x += step.x;
  pose_.position.y += step.y;
  pose_.position.z += step.z;

}

void Localizer::on_imu(const sensor_msgs::msg::Imu & imu)
{
  // The compass gives the heading directly
  pose_.orientation = imu.orientation;
}

void Localizer::on_sonar(const sensor_msgs::msg::LaserScan & scan, const nav_msgs::msg::OccupancyGrid & map)
{
  if (map.data.empty()) {
    return;
  }
  const double yaw = yaw_of(pose_);
  const double x = pose_.position.x;
  const double y = pose_.position.y;

  // Try small shifts of the position around the current estimate; keep the one the map explains best
  const int steps = static_cast<int>(std::round(kSearchRadius / kSearchStep));
  const int score_here = count_agreeing_beams(scan, map, x, y, yaw);
  int best_score = score_here;
  double best_dx = 0.0;
  double best_dy = 0.0;
  for (int i = -steps; i <= steps; ++i) {
    for (int j = -steps; j <= steps; ++j) {
      const double dx = i * kSearchStep;
      const double dy = j * kSearchStep;
      const int score = count_agreeing_beams(scan, map, x + dx, y + dy, yaw);
      if (score > best_score) {
        best_score = score;
        best_dx = dx;
        best_dy = dy;
      }
    }
  }

  // Not convinced: keep the estimate as it is
  if (best_score < kMinInliers || best_score - score_here < kMinGain) {
    return;
  }

  // Move part of the way toward the best guess (only x and y: the sonar is a flat ring, it says
  // nothing about height)
  pose_.position.x += kNudge * best_dx;
  pose_.position.y += kNudge * best_dy;
}

void Localizer::on_depth(const sensor_msgs::msg::Image & depth, const sensor_msgs::msg::CameraInfo & camera_info)
{
  const double fy = camera_info.k[4];
  const double cy = camera_info.k[5];
  if (fy <= 0.0) {
    return;
  }
  // How far below the camera each pixel in the bottom part of the image hits something
  std::vector<double> drops;
  for (int v = static_cast<int>(cy) + 8; v < static_cast<int>(depth.height); ++v) {
    for (int u = 0; u < static_cast<int>(depth.width); ++u) {
      const float d = depth_at(depth, u, v);
      if (std::isfinite(d) && d > 0.0f) {
        drops.push_back(d * (v + 0.5 - cy) / fy);
      }
    }
  }
  if (drops.empty()) {
    return;
  }
  // The floor is the lowest surface: the biggest drops. Average the ones near the biggest
  const double biggest = *std::max_element(drops.begin(), drops.end());
  double sum = 0.0;
  int count = 0;
  for (const double drop : drops) {
    if (drop > biggest - kFloorBand) {
      sum += drop;
      ++count;
    }
  }
  if (count < kMinFloorPixels) {
    return;  // not enough floor in view (looking at a wall or over furniture)
  }
  const double measured_z = sum / count - kCameraUp;
  if (std::abs(measured_z - pose_.position.z) > kMaxHeightJump) {
    return;  // more likely a big flat table top than the floor: the real drift is slow
  }
  pose_.position.z += kHeightNudge * (measured_z - pose_.position.z);
}

}  // namespace ghost_brain
