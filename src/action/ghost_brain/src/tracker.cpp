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

#include "ghost_brain/tracker.hpp"

#include <cstring>
#include <limits>

namespace ghost_brain
{

float depth_at(const sensor_msgs::msg::Image & depth, int u, int v)
{
  if (u < 0 || v < 0 || u >= static_cast<int>(depth.width) || v >= static_cast<int>(depth.height)) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  float value = 0.0f;
  std::memcpy(&value, &depth.data[v * depth.step + u * sizeof(float)], sizeof(float));
  return value;
}

void Tracker::reset()
{
  targets_.clear();
}

void Tracker::on_detections(
  const vision_msgs::msg::Detection2DArray & detections,
  const sensor_msgs::msg::Image & depth,
  const sensor_msgs::msg::CameraInfo & camera_info,
  const geometry_msgs::msg::Pose & ghost_pose)
{
  // TODO(hackathon): work out where each detection is in the map, and keep track of them in targets_
  (void)detections;
  (void)depth;
  (void)camera_info;
  (void)ghost_pose;
}

}  // namespace ghost_brain
