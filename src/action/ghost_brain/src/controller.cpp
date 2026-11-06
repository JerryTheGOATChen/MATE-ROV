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

#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Vector3.hpp"

namespace ghost_brain
{

geometry_msgs::msg::Twist Controller::compute_velocity(const geometry_msgs::msg::Pose & pose, const Path & path)
{
  geometry_msgs::msg::Twist command;
  if (path.empty()) {
    previous_path_.clear();
    next_waypoint_ = 0;
    return command;
  }

  auto distance = [&](const geometry_msgs::msg::Point & point) {
    return std::hypot(
      point.x - pose.position.x, point.y - pose.position.y, point.z - pose.position.z);
  };
  // Only initialize from the nearest waypoint when the planner supplies a new route.
  // Otherwise keep progress, so a climb cannot send us back to the lower waypoint.
  if (path != previous_path_) {
    previous_path_ = path;
    next_waypoint_ = 0;
    for (size_t i = 1; i < path.size(); ++i) {
      if (distance(path[i]) < distance(path[next_waypoint_])) {
        next_waypoint_ = i;
      }
    }
  }
  auto & next = next_waypoint_;
  while (next + 1 < path.size() && distance(path[next]) <= 0.15) {
    ++next;
  }
  if (next + 1 == path.size() && distance(path[next]) <= 0.05) {
    return command;
  }

  const auto & waypoint = path[next];
  const tf2::Vector3 map_error(
    waypoint.x - pose.position.x, waypoint.y - pose.position.y, waypoint.z - pose.position.z);
  const auto & q = pose.orientation;
  const tf2::Quaternion orientation(q.x, q.y, q.z, q.w);
  const auto error = tf2::quatRotate(orientation.inverse(), map_error);

  constexpr double kp = 4.0;
  command.linear.x = kp * error.x();
  command.linear.y = kp * error.y();
  command.linear.z = kp * error.z();
  // Scale together to preserve the direction while respecting each axis's limit.
  const double scale = std::max({
    1.0, std::abs(command.linear.x) / 1.0,
    std::abs(command.linear.y) / 0.6, std::abs(command.linear.z) / 0.6});
  command.linear.x /= scale;
  command.linear.y /= scale;
  command.linear.z /= scale;
  if (std::hypot(error.x(), error.y()) > 0.05) {
    command.angular.z = std::clamp(1.5 * std::atan2(error.y(), error.x()), -1.5, 1.5);
  }
  return command;
}

}  // namespace ghost_brain
