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

namespace ghost_brain
{

void Localizer::reset(const geometry_msgs::msg::Pose & start)
{
  pose_ = start;
}

void Localizer::on_velocity(const geometry_msgs::msg::TwistStamped & velocity)
{
  // The assumption is that on-average the time difference between each velocity pubish
  // is relatively equal. 
  rclcpp::Time current_time{velocity.header.stamp}; 
  double dt = (previous_time - current_time).seconds(); 
  tf2::Quaternion orientation;
  tf2::fromMsg(pose_.orientation, orientation);

  tf2::Vector3 body_velocity(
    velocity.twist.linear.x,
    velocity.twist.linear.y,
    velocity.twist.linear.z);

  tf2::Vector3 global_velocity = tf2::quatRotate(orientation, body_velocity);  
  pose_.position.x += global_velocity.x() * dt;
  pose_.position.y += global_velocity.y() * dt;
  pose_.position.z += global_velocity.z() * dt;

  previous_time = current_time;
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
