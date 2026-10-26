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

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace ghost_brain
{

/// A person or piece of candy the ghost has seen, and where it thinks it is
struct TrackedTarget
{
  geometry_msgs::msg::Point position;
  bool is_person{false};
  rclcpp::Time last_seen;
};

/// Where the camera sits on the ghost: this far forward, left and up from its center (m)
constexpr double kCameraForward = 0.25;
constexpr double kCameraLeft = 0.0;
constexpr double kCameraUp = 0.1;

/**
 * @brief The depth at a pixel of a depth image, in meters.
 *
 * Depth is measured along the camera's axis, not along the ray to the pixel. It is infinite where
 * nothing is in range, and NaN outside the image.
 */
float depth_at(const sensor_msgs::msg::Image & depth, int u, int v);

/**
 * @brief Keeps track of where the people and candy are. This is yours to write!
 *
 * Nobody tells the ghost where anything is. Its camera faces forward with a 120 degree view, and
 * every frame (10 Hz) comes with:
 *  - /camera/detections: a box in the image around each person's head and each candy in view. It
 *    says what it is ("person" or "candy" in results[0].hypothesis.class_id), but not where or
 *    which one
 *  - /camera/depth: how far away every pixel is (32-bit floats, in meters)
 *  - /camera/camera_info: the camera's focal length and center in `k`
 *
 * To place a detection in the house: read the depth at its box, turn the pixel and depth into a
 * point in the camera's frame (the optical frame: x right, y down, z forward), then into the ghost's
 * frame (the camera is mounted kCamera* from its center), then into the map with the ghost's pose.
 * People are collected at their head, which is what their box is around. The depth there is to
 * the front of their head, about 0.2 m nearer than its center. Far away, a box can be under a pixel
 * wide, so its center pixel may miss: the nearest depth inside the box is safer.
 *
 * Some ideas once that works:
 *  - Match each detection to a target you already know about, so you don't count it twice
 *  - Track people's velocity, so you can predict where they're going
 *  - Forget targets when you collect them (the score in /haunted_house/state goes up), or when you
 *    look where one should be and it isn't there
 */
class Tracker
{
public:
  /// Forget everything. Called at the start of every game
  void reset();

  void on_detections(
    const vision_msgs::msg::Detection2DArray & detections,
    const sensor_msgs::msg::Image & depth,
    const sensor_msgs::msg::CameraInfo & camera_info,
    const geometry_msgs::msg::Pose & ghost_pose);

  const std::vector<TrackedTarget> & targets() const { return targets_; }

private:
  std::vector<TrackedTarget> targets_;
};

}  // namespace ghost_brain
