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

#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "haunted_house_msgs/msg/box_array.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace ghost_brain
{

/**
 * @brief Estimates the ghost's pose from velocity, compass, and sonar/map alignment.
 *
 * The sonar sweeps in segments. Keep a short rolling history of sonar hits so
 * scan matching has information from more than one direction.
 */
class Localizer
{
public:
  void reset(const geometry_msgs::msg::Pose & start);
  void on_velocity(const geometry_msgs::msg::TwistStamped & velocity);
  void on_imu(const sensor_msgs::msg::Imu & imu);
  void on_sonar(const sensor_msgs::msg::LaserScan & scan, const nav_msgs::msg::OccupancyGrid & map);

  void on_depth(
    const sensor_msgs::msg::Image & depth, const sensor_msgs::msg::CameraInfo & camera_info,
    const haunted_house_msgs::msg::BoxArray & obstacles);

  geometry_msgs::msg::Pose pose() const { return pose_; }

private:
  struct SonarHit
  {
    double x;      // Estimated world X of detected surface
    double y;      // Estimated world Y of detected surface
    double stamp;  // Scan time in seconds
  };

  geometry_msgs::msg::Pose pose_;
  double previous_velocity_time_ = 0.0;
  bool have_previous_velocity_ = false;
  bool initialized_ = false;
  std::vector<SonarHit> sonar_hits_;
};

}  // namespace ghost_brain