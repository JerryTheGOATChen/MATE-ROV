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

#include "ghost_brain/controller.hpp"

#include <algorithm>
#include <cmath>

namespace ghost_brain
{

namespace
{

// Hold horizontal pursuit speed closer to moving targets (gains in 1/s).
constexpr double kHorizontalGain = 2.4;
constexpr double kVerticalGain = 1.2;
constexpr double kYawGain = 1.5;

constexpr double kMaxForwardSpeed = 1.0;
constexpr double kMaxSidewaysSpeed = 0.6;
constexpr double kMaxVerticalSpeed = 0.6;
constexpr double kMaxYawRate = 1.5;

// Don't cut corners: only advance to a subsequent waypoint once very close.
constexpr double kWaypointTolerance = 0.10;
// Planner may end as far as 0.45 m from candy; leave room for 0.5 m scoring.
constexpr double kFinalTolerance = 0.03;
constexpr double kMinHeadingDistance = 0.10;

double distance(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b)
{
  return std::hypot(a.x - b.x, a.y - b.y, a.z - b.z);
}

}  // namespace

geometry_msgs::msg::Twist Controller::compute_velocity(
  const geometry_msgs::msg::Pose & pose,
  const Path & path)
{
  geometry_msgs::msg::Twist command{};

  if (path.empty()) {
    return command;  // No safe route.
  }

  // The planner returns {start, ..., goal}. Skip the recorded start position,
  // since the robot has already left it (and plans are reused between replans).
  size_t waypoint_index = path.size() > 1 ? 1 : 0;

  // Progress through waypoints, preserving turns around obstacles.
  while (waypoint_index + 1 < path.size() &&
         distance(pose.position, path[waypoint_index]) < kWaypointTolerance) {
    ++waypoint_index;
  }

  const auto & target = path[waypoint_index];
  const double dx = target.x - pose.position.x;
  const double dy = target.y - pose.position.y;
  const double dz = target.z - pose.position.z;

  // Stop only at the final waypoint, not at intermediate corners.
  if (waypoint_index == path.size() - 1 &&
      std::hypot(dx, dy, dz) < kFinalTolerance) {
    return command;
  }

  const auto & q = pose.orientation;
  const double yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  // Position error is in the map frame; /cmd_vel expects the ghost's body frame.
  const double forward_error = dx * std::cos(yaw) + dy * std::sin(yaw);
  const double left_error = -dx * std::sin(yaw) + dy * std::cos(yaw);

  command.linear.x = std::clamp(
    kHorizontalGain * forward_error, -kMaxForwardSpeed, kMaxForwardSpeed);
  command.linear.y = std::clamp(
    kHorizontalGain * left_error, -kMaxSidewaysSpeed, kMaxSidewaysSpeed);
  command.linear.z = std::clamp(
    kVerticalGain * dz, -kMaxVerticalSpeed, kMaxVerticalSpeed);

  // Face toward the current waypoint while translating in any direction.
  // Use shortest signed angular difference to avoid a 2*pi turn at +/-pi.
  if (std::hypot(dx, dy) > kMinHeadingDistance) {
    const double target_yaw = std::atan2(dy, dx);
    const double yaw_error = std::atan2(
      std::sin(target_yaw - yaw), std::cos(target_yaw - yaw));
    command.angular.z = std::clamp(
      kYawGain * yaw_error, -kMaxYawRate, kMaxYawRate);
  }

  return command;
}

}  // namespace ghost_brain