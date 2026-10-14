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

#include "rov_kinematic_sim/rov_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <Eigen/Geometry>

#include "angles/angles.h"

namespace rov_kinematic_sim
{
namespace
{

/// Velocity over ground in the world frame: thrust-driven velocity plus the water current.
Eigen::Vector3d ground_velocity(const VehicleState & state, const VehicleModel & model)
{
  return Eigen::AngleAxisd(state.yaw, Eigen::Vector3d::UnitZ()) * state.velocity.linear + model.water_current;
}

}  // namespace

VehicleState step(const VehicleState & state, const BodyVelocity & command, const VehicleModel & model, double dt)
{
  // The thrusters saturate, and the vehicle's velocity approaches what they can deliver
  const Eigen::Vector3d linear_target =
    command.linear.cwiseMax(-model.max_linear_speed).cwiseMin(model.max_linear_speed);
  const double yaw_rate_target = std::clamp(command.yaw_rate, -model.max_yaw_rate, model.max_yaw_rate);
  const double blend = model.response_time > 0.0 ? 1.0 - std::exp(-dt / model.response_time) : 1.0;

  VehicleState next = state;
  next.velocity.linear += blend * (linear_target - state.velocity.linear);
  next.velocity.yaw_rate += blend * (yaw_rate_target - state.velocity.yaw_rate);

  // Move along the current heading, then turn
  next.position += ground_velocity(next, model) * dt;
  next.yaw = angles::normalize_angle(state.yaw + next.velocity.yaw_rate * dt);

  // At the surface the vehicle floats: it stops there and cannot keep rising
  if (next.position.z() > model.surface_z) {
    next.position.z() = model.surface_z;
    next.velocity.linear.z() = std::min(next.velocity.linear.z(), 0.0);
  }

  return next;
}

}  // namespace rov_kinematic_sim
