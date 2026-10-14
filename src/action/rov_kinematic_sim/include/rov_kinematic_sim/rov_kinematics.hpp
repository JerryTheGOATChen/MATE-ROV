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

namespace rov_kinematic_sim
{

/// Velocity in the vehicle (base_link) frame: surge, sway and heave, plus yaw rate.
struct BodyVelocity
{
  Eigen::Vector3d linear{Eigen::Vector3d::Zero()};
  double yaw_rate{0.0};
};

/// State of an ROV that holds itself level in roll and pitch.
struct VehicleState
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};  // m, world frame; z is up, the surface is at surface_z
  double yaw{0.0};  // rad
  BodyVelocity velocity;  // through the water, driven by the thrusters
};

/**
 * @brief Simplified ROV dynamics.
 *
 * The vehicle's velocity through the water follows the commanded velocity with a first-order lag,
 * which stands in for thruster spin-up, drag and added mass. A uniform water current carries the
 * vehicle, and it cannot rise above the surface.
 */
struct VehicleModel
{
  double response_time{0.3};  // s, time constant of the velocity lag; zero responds instantly
  Eigen::Vector3d max_linear_speed{1.0, 0.6, 0.6};  // m/s per vehicle axis, the thrusters' limits
  double max_yaw_rate{1.0};  // rad/s
  Eigen::Vector3d water_current{Eigen::Vector3d::Zero()};  // m/s, world frame
  double surface_z{0.0};  // m
};

/// Advance `state` by `dt` seconds while the thrusters track `command`.
VehicleState step(const VehicleState & state, const BodyVelocity & command, const VehicleModel & model, double dt);

}  // namespace rov_kinematic_sim
