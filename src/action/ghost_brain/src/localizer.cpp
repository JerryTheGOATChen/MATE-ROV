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
#include <cmath>

namespace ghost_brain
{

void Localizer::reset(const geometry_msgs::msg::Pose & start)
{
  pose_ = start;

  previous_velocity_time_ = 0.0;
  have_previous_velocity_ = false;
}

void Localizer::on_velocity(const geometry_msgs::msg::TwistStamped & velocity)
{
  // TODO(hackathon): move pose_.position by the velocity since the last message
  const double current_time = static_cast<double>(velocity.header.stamp.sec)+static_cast<double>(velocity.header.stamp.nanosec) * 1e-9;

  if (!have_previous_velocity_) {
    previous_velocity_time_ = current_time;
    have_previous_velocity_ = true;
    return;
  }

  const double dt = current_time - previous_velocity_time_;

  previous_velocity_time_ = current_time;

  if (dt <= 0.0) {
    return;
  }

  //Extract Velocity and Orientation

  const double vx = velocity.twist.linear.x;
  const double vy = velocity.twist.linear.y;  
  const double vz = velocity.twist.linear.z;

  const auto & q = pose_.orientation;

  // Convert quaternion to yaw
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  const double world_vx = vx * std::cos(yaw) - vy * std::sin(yaw);
  const double world_vy = vx * std::sin(yaw) + vy * std::cos(yaw);
  
  pose_.position.x += world_vx * dt;
  pose_.position.y += world_vy * dt;
  pose_.position.z += vz * dt;
}

void Localizer::on_imu(const sensor_msgs::msg::Imu & imu)
{
  // The compass gives the heading directly
  pose_.orientation = imu.orientation;
}

void Localizer::on_sonar(const sensor_msgs::msg::LaserScan & scan, const nav_msgs::msg::OccupancyGrid & map)
{
  // TODO(hackathon): bonus! Use the walls the sonar sees to correct the drift
  (void)scan;
  (void)map;
}

}  // namespace ghost_brain
