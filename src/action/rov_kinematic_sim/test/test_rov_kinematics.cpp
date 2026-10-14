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

#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "rov_kinematic_sim/rov_kinematics.hpp"

using Catch::Matchers::WithinAbs;
using rov_kinematic_sim::BodyVelocity;
using rov_kinematic_sim::step;
using rov_kinematic_sim::VehicleModel;
using rov_kinematic_sim::VehicleState;

namespace
{

constexpr double kTolerance = 1e-9;
constexpr double kDt = 0.02;  // s

VehicleModel instant_model()
{
  VehicleModel model;
  model.response_time = 0.0;
  return model;
}

BodyVelocity body_velocity(double x, double y, double z, double yaw_rate = 0.0)
{
  BodyVelocity velocity;
  velocity.linear = Eigen::Vector3d(x, y, z);
  velocity.yaw_rate = yaw_rate;
  return velocity;
}

/// Hold `command` for `duration` seconds.
VehicleState run(VehicleState state, const BodyVelocity & command, const VehicleModel & model, double duration)
{
  for (double t = 0.0; t < duration - kDt / 2; t += kDt) {
    state = step(state, command, model, kDt);
  }
  return state;
}

VehicleState submerged()
{
  VehicleState state;
  state.position.z() = -2.0;
  return state;
}

}  // namespace

TEST_CASE("The vehicle holds still in still water without a command", "[rov_kinematics]")
{
  const VehicleState state = run(submerged(), BodyVelocity{}, VehicleModel{}, 1.0);
  REQUIRE_THAT((state.position - submerged().position).norm(), WithinAbs(0.0, kTolerance));
}

TEST_CASE("The vehicle's velocity lags the command by the response time", "[rov_kinematics]")
{
  VehicleModel model;
  model.response_time = 0.3;

  // After one time constant, the velocity has covered 1 - 1/e of the step
  const VehicleState state = run(submerged(), body_velocity(0.5, 0.0, 0.0), model, 0.3);
  REQUIRE_THAT(state.velocity.linear.x(), WithinAbs(0.5 * (1.0 - std::exp(-1.0)), 1e-6));
}

TEST_CASE("The vehicle moves along its heading", "[rov_kinematics]")
{
  VehicleState start = submerged();
  start.yaw = M_PI_2;

  // Surging while facing +y moves the vehicle along +y in the world
  const VehicleState state = run(start, body_velocity(0.5, 0.0, 0.0), instant_model(), 2.0);
  REQUIRE_THAT(state.position.x(), WithinAbs(0.0, 1e-6));
  REQUIRE_THAT(state.position.y(), WithinAbs(1.0, 1e-6));
}

TEST_CASE("The thrusters saturate", "[rov_kinematics]")
{
  const VehicleModel model = instant_model();
  const VehicleState state = step(submerged(), body_velocity(5.0, -5.0, 5.0, -5.0), model, kDt);

  REQUIRE_THAT(state.velocity.linear.x(), WithinAbs(model.max_linear_speed.x(), kTolerance));
  REQUIRE_THAT(state.velocity.linear.y(), WithinAbs(-model.max_linear_speed.y(), kTolerance));
  REQUIRE_THAT(state.velocity.linear.z(), WithinAbs(model.max_linear_speed.z(), kTolerance));
  REQUIRE_THAT(state.velocity.yaw_rate, WithinAbs(-model.max_yaw_rate, kTolerance));
}

TEST_CASE("A water current carries the vehicle", "[rov_kinematics]")
{
  VehicleModel model;
  model.water_current = Eigen::Vector3d(0.1, -0.05, 0.02);

  const VehicleState state = run(submerged(), BodyVelocity{}, model, 2.0);
  REQUIRE_THAT((state.position - submerged().position - 2.0 * model.water_current).norm(), WithinAbs(0.0, 1e-6));
}

TEST_CASE("The vehicle cannot rise above the surface", "[rov_kinematics]")
{
  const VehicleModel model = instant_model();
  const VehicleState state = run(submerged(), body_velocity(0.0, 0.0, 0.5), model, 10.0);

  REQUIRE_THAT(state.position.z(), WithinAbs(model.surface_z, kTolerance));
  REQUIRE(state.velocity.linear.z() == 0.0);
}

TEST_CASE("Yaw wraps around at +/-pi", "[rov_kinematics]")
{
  VehicleState start = submerged();
  start.yaw = 3.0;

  const VehicleState state = run(start, body_velocity(0.0, 0.0, 0.0, 0.5), instant_model(), 1.0);
  REQUIRE_THAT(state.yaw, WithinAbs(3.5 - 2.0 * M_PI, 1e-6));
}
