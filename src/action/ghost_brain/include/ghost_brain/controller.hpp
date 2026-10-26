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

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ghost_brain/planner.hpp"

namespace ghost_brain
{

/**
 * @brief Turns the planned route into velocity commands. This is yours to write!
 *
 * The ghost flies like the ROV: the command is in the ghost's own frame, with linear.x forward,
 * linear.y to its left, linear.z up, and angular.z its turn rate (counterclockwise from above).
 * It can go at most 1.0 m/s forward, 0.6 m/s sideways and up or down, and turn 1.5 rad/s, and
 * it takes about 0.3 s to respond.
 *
 * The pose is in the map frame. Its yaw is in the orientation quaternion: tf2::getYaw() from
 * "tf2/utils.hpp" pulls it out.
 *
 * Some ideas, from simplest to fanciest:
 *  - Turn toward the next point, then fly at it. Climb or dive at the same time
 *  - A proportional controller, then a PID
 *  - Pure pursuit: chase a point a little way down the route, so corners come out smooth
 *  - Move sideways, so the ghost does not have to face where it is going (try it backwards!)
 */
class Controller
{
public:
  /// @return the velocity to send now, in the ghost's own frame. Return all zeros to stop
  geometry_msgs::msg::Twist compute_velocity(const geometry_msgs::msg::Pose & pose, const Path & path);
};

}  // namespace ghost_brain
