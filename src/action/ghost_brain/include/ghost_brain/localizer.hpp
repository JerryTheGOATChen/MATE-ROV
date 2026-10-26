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
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace ghost_brain
{

/**
 * @brief Works out where the ghost is. This is yours to write!
 *
 * Nobody tells the ghost its position. It knows where it started (the house publishes the start
 * pose), its heading from a compass, and its velocity from something like a DVL:
 *  - /ghost/velocity: forward, left and up speed in the ghost's own frame, and its turn rate, at 25 Hz.
 *    It is a little noisy, and has a small bias that changes every game, so adding it up drifts
 *  - /ghost/imu: the heading in `orientation` (yaw only), at 25 Hz
 *  - /sonar/scan: a sonar beam sweeping a full circle once a second, a tenth of a turn per message.
 *    Each range is the distance to the nearest thing in that direction, at the ghost's height
 *
 * Some ideas, from simplest to fanciest:
 *  - Dead reckoning: turn each velocity into the map frame with the heading, and add it up over time
 *  - Spot the drift: the sonar sees the walls, and /map says where the walls really are
 *  - A particle filter, or scan matching, to correct the position with the sonar
 */
class Localizer
{
public:
  /// Start again from a known pose. Called at the start of every game
  void reset(const geometry_msgs::msg::Pose & start);

  void on_velocity(const geometry_msgs::msg::TwistStamped & velocity);
  void on_imu(const sensor_msgs::msg::Imu & imu);
  void on_sonar(const sensor_msgs::msg::LaserScan & scan, const nav_msgs::msg::OccupancyGrid & map);

  /// The best guess of where the ghost is, in the map frame
  geometry_msgs::msg::Pose pose() const { return pose_; }

private:
  geometry_msgs::msg::Pose pose_;
};

}  // namespace ghost_brain
