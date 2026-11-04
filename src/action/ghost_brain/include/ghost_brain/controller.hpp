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
/// One PID loop: its gains plus the memory it needs between ticks
struct Pid
{
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  double integral = 0.0;     // running sum of error * dt
  double prev_error = 0.0;   // error from the last tick, for the D term
  bool has_prev = false;     // false on the first tick after a reset (no D spike)

  /// @return kp*error + ki*integral + kd*derivative, using dt seconds since the last tick
  double update(double error, double dt);

  /// Forget the past: zero the integral and previous error
  void reset();
};

class Controller
{
public:
  /// @return the velocity to send now, in the ghost's own frame. Return all zeros to stop
  geometry_msgs::msg::Twist compute_velocity(const geometry_msgs::msg::Pose & pose, const Path & path);

private:
  /// Clear the PID memory. Call when the path is empty or the target changes
  void reset();

  static constexpr double kDt = 0.05;            // 20 Hz control loop
  static constexpr double kLookahead = 0.6;      // m, how far down the path to aim
  static constexpr double kTolerance = 0.5;      // m, close enough to the final point

  Pid heading_pid_{2.0, 0.0, 0.3};   // heading error -> angular.z
  Pid altitude_pid_{1.5, 0.0, 0.2};  // z error -> linear.z
  // Pid speed_pid_;  // optional: distance -> linear.x

  // Used to notice a new target, so the PID memory can be reset
  geometry_msgs::msg::Point last_goal_;
  bool has_last_goal_ = false;


};

}  // namespace ghost_brain

//try to add strafe running
