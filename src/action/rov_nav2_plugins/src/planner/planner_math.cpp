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
#include "rov_nav2_plugins/planner/planner_math.hpp"

#include <cstddef>
#include <stdexcept>
#include <vector>

#include "angles/angles.h"

namespace rov_nav2_plugins
{
namespace
{

template <typename Point>
double polyline_length(const std::vector<Point> & points)
{
  double length = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    length += (points[i] - points[i - 1]).norm();
  }
  return length;
}

}  // namespace

std::vector<Pose4D> add_depth(
  const std::vector<Eigen::Vector2d> & xy_path, const Pose4D & start, const Pose4D & goal, bool z_before_xy)
{
  if (xy_path.empty()) {
    throw std::invalid_argument("xy_path must not be empty");
  }

  // The 2D path with its ends replaced by the exact start and goal
  std::vector<Eigen::Vector2d> xy{start.position.head<2>()};
  for (std::size_t i = 1; i + 1 < xy_path.size(); ++i) {
    xy.push_back(xy_path[i]);
  }
  xy.push_back(goal.position.head<2>());
  const double xy_length = polyline_length(xy);

  std::vector<Eigen::Vector3d> points{start.position};
  double z = start.position.z();
  if (z_before_xy || xy_length == 0.0) {
    z = goal.position.z();
    points.emplace_back(start.position.x(), start.position.y(), z);
  }
  const double z_change = goal.position.z() - z;
  double xy_travelled = 0.0;
  for (std::size_t i = 1; i < xy.size(); ++i) {
    xy_travelled += (xy[i] - xy[i - 1]).norm();
    const double fraction = xy_length > 0.0 ? xy_travelled / xy_length : 1.0;
    points.emplace_back(xy[i].x(), xy[i].y(), z + z_change * fraction);
  }

  const double length = polyline_length(points);
  const double yaw_change = angles::shortest_angular_distance(start.yaw, goal.yaw);
  std::vector<Pose4D> path{start};
  double travelled = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    travelled += (points[i] - points[i - 1]).norm();
    const double fraction = length > 0.0 ? travelled / length : 1.0;
    path.push_back({points[i], angles::normalize_angle(start.yaw + yaw_change * fraction)});
  }
  path.back() = goal;
  return path;
}

}  // namespace rov_nav2_plugins
