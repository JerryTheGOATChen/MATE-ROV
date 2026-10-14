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

#include <Eigen/Core>

#include "geometry_msgs/msg/pose.hpp"

namespace rov_nav2_plugins
{

/**
 * @brief Position and heading of a vehicle that holds itself level.
 *
 * An ROV is passively stable in roll and pitch, so navigation only plans and controls
 * four degrees of freedom: x, y, z and yaw.
 */
struct Pose4D
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  double yaw{0.0};
};

/// Convert a pose message, dropping roll and pitch.
Pose4D to_pose4d(const geometry_msgs::msg::Pose & pose);

/// Convert to a pose message with a yaw-only orientation.
geometry_msgs::msg::Pose to_pose_msg(const Pose4D & pose);

}  // namespace rov_nav2_plugins
