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
#include <cmath>

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
  //Camera Info
    double fx = camera_info.k[0];
    double fy = camera_info.k[4];

    double cx = camera_info.k[2];
    double cy = camera_info.k[5];

    const auto &q = ghost_pose.orientation;

    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    rclcpp::Time current_time(detections.header.stamp);

    for (auto it = targets_.begin(); it != targets_.end();){
        if (it->is_person && (current_time - it->last_seen).seconds() > 5){
            it = targets_.erase(it);
        }else{
            ++it;
        }
    }

  for (const auto &detection : detections.detections){
    TrackedTarget target; 

    bool isperson = false;

    //B-Box Info
    double u = detection.bbox.center.position.x;
    double v = detection.bbox.center.position.y;
    float distance = depth_at(depth, static_cast<int>(std::round(u)), static_cast<int>(std::round(v)));

    if (!std::isfinite(distance) || distance <= 0.0f) {
        continue;
    }
    
    //Coordinates To Camera
    double xc = ((u-cx)*distance)/fx;
    double yc = ((v-cy)*distance)/fy;
    double zc = distance;

    //Cordinates To Ghost
    double xb = zc + kCameraForward;
    double yb = -xc + kCameraLeft;
    double zb = -yc + kCameraUp;

    //Coordinates to World
    const double tx = ghost_pose.position.x + xb * std::cos(yaw) - yb * std::sin(yaw);
    const double ty = ghost_pose.position.y + xb * std::sin(yaw) + yb * std::cos(yaw);
    const double tz = ghost_pose.position.z + zb;

    //Person Check
    if (detection.results[0].hypothesis.class_id == "person"){
      isperson = true;
    }

    target.position.x = tx;
    target.position.y = ty;
    target.position.z = tz;
    target.is_person = isperson; 
    target.last_seen = rclcpp::Time(detections.header.stamp);

    bool target_changed = false;

    for (auto &existing_target : targets_){
      if (!target.is_person && !existing_target.is_person){
        if (std::sqrt(std::pow((existing_target.position.x - target.position.x),2)
            + std::pow((existing_target.position.y - target.position.y),2)
            + std::pow((existing_target.position.z - target.position.z),2)) < 0.5){
              target_changed = true;
              existing_target.position.x = target.position.x;
              existing_target.position.y = target.position.y;
              existing_target.position.z = target.position.z;
              existing_target.last_seen = target.last_seen;
              break;
            }
      }else if (target.is_person && existing_target.is_person){
        if (std::sqrt(std::pow((existing_target.position.x - target.position.x),2)
            + std::pow((existing_target.position.y - target.position.y),2)
            + std::pow((existing_target.position.z - target.position.z),2)) < 1){
              target_changed = true;
              existing_target.position.x = target.position.x;
              existing_target.position.y = target.position.y;
              existing_target.position.z = target.position.z;
              existing_target.last_seen = target.last_seen;
              break;
            }
      }
    }
    if (!target_changed){
      targets_.push_back(target);
    }
    
  }
}

}  // namespace ghost_brain
