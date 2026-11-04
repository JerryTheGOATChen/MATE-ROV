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
  has_last_velocity_stamp_ = false;
}

void Localizer::on_velocity(const geometry_msgs::msg::TwistStamped & velocity)
{
  // TODO(hackathon): move pose_.position by the velocity since the last message
  if (!has_last_velocity_stamp_) {
    last_velocity_stamp_ = velocity.header.stamp;
    has_last_velocity_stamp_ = true;
    return;
  }

  //finds the change in time
  double dt = (velocity.header.stamp.sec - last_velocity_stamp_.sec) + ((static_cast<double>(velocity.header.stamp.nanosec) - static_cast<double>(last_velocity_stamp_.nanosec))/1e9);
  
  //makes sure that it isnt an invalid value
  if(dt <= 0){
    return;
  }

  //this gives us the heading in radieans
  double yaw = 2.0 * std::atan2(pose_.orientation.z, pose_.orientation.w);
  double vx_world = (velocity.twist.linear.x * std::cos(yaw)) - (velocity.twist.linear.y * std::sin(yaw));
  double vy_world = (velocity.twist.linear.x * std::sin(yaw)) + (velocity.twist.linear.y * std::cos(yaw));

  pose_.position.x += vx_world*dt;
  pose_.position.y += vy_world*dt;
  pose_.position.z += velocity.twist.linear.z*dt;

  last_velocity_stamp_ = velocity.header.stamp;

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
