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

#include <algorithm>
#include <cstring>
#include <limits>
#include <cmath>

namespace ghost_brain
{

// NEW: Check whether two detection bounding boxes overlap.
// Assumes the detection bounding boxes are axis-aligned.
namespace
{

bool boxes_overlap(
  const vision_msgs::msg::BoundingBox2D & a,
  const vision_msgs::msg::BoundingBox2D & b)
{
  const double dx = std::abs(
    a.center.position.x - b.center.position.x);

  const double dy = std::abs(
    a.center.position.y - b.center.position.y);

  return (
    dx < (a.size_x + b.size_x) / 2.0 &&
    dy < (a.size_y + b.size_y) / 2.0
  );
}

}  // namespace


float depth_at(const sensor_msgs::msg::Image & depth, int u, int v)
{
  if (
    u < 0 || v < 0 ||
    u >= static_cast<int>(depth.width) ||
    v >= static_cast<int>(depth.height))
  {
    return std::numeric_limits<float>::quiet_NaN();
  }

  float value = 0.0f;

  std::memcpy(
    &value,
    &depth.data[v * depth.step + u * sizeof(float)],
    sizeof(float));

  return value;
}


void Tracker::reset()
{
  targets_.clear();
}


void Tracker::reset_people(const geometry_msgs::msg::Point & position, double radius)
{
  targets_.erase(
    std::remove_if(targets_.begin(), targets_.end(),
      [&](const TrackedTarget & target) {
        return target.is_person && std::hypot(
          target.position.x - position.x, target.position.y - position.y,
          target.position.z - position.z) <= radius;
      }),
    targets_.end());
}

bool Tracker::mark_completed(
  const geometry_msgs::msg::Point & position, bool is_person, double max_distance)
{
  TrackedTarget * nearest = nullptr;
  double best_distance = max_distance;
  for (auto & target : targets_) {
    if (target.completed || target.is_person != is_person) {
      continue;
    }
    const double d = std::hypot(
      target.position.x - position.x, target.position.y - position.y,
      target.position.z - position.z);
    if (d <= best_distance) {
      nearest = &target;
      best_distance = d;
    }
  }
  if (!nearest) {
    return false;
  }
  nearest->completed = true;
  return true;
}

bool Tracker::remove_target(const geometry_msgs::msg::Point & position, bool is_person)
{
  auto nearest = targets_.end();
  double best_distance = 0.3;
  for (auto it = targets_.begin(); it != targets_.end(); ++it) {
    if (it->is_person != is_person || it->completed) {
      continue;
    }
    const double d = std::hypot(
      it->position.x - position.x, it->position.y - position.y,
      it->position.z - position.z);
    if (d <= best_distance) {
      nearest = it;
      best_distance = d;
    }
  }
  if (nearest == targets_.end()) {
    return false;
  }
  targets_.erase(nearest);
  return true;
}

void Tracker::on_detections(
  const vision_msgs::msg::Detection2DArray & detections,
  const sensor_msgs::msg::Image & depth,
  const sensor_msgs::msg::CameraInfo & camera_info,
  const geometry_msgs::msg::Pose & ghost_pose)
{
  // Camera information
  const double fx = camera_info.k[0];
  const double fy = camera_info.k[4];

  const double cx = camera_info.k[2];
  const double cy = camera_info.k[5];

  const auto & q = ghost_pose.orientation;

  const double yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  rclcpp::Time current_time(detections.header.stamp);

  // Keep last-known person locations as fallback goals when no live targets remain.
  for (auto it = targets_.begin(); it != targets_.end();) {
    if (
      it->is_person && !it->completed &&
      (current_time - it->last_seen).seconds() > 30.0)
    {
      it = targets_.erase(it);
    } else {
      ++it;
    }
  }

  // Process detections
  for (const auto & detection : detections.detections) {

    // Safety check: a detection might not have a classification.
    if (detection.results.empty()) {
      continue;
    }

    const std::string & class_id =
      detection.results[0].hypothesis.class_id;

    const bool isperson = (class_id == "person");
    const bool iscandy = (class_id == "candy");

    // Ignore unknown classes.
    if (!isperson && !iscandy) {
      continue;
    }

    if (iscandy) {

      bool overlaps_person = false;

      // Compare this candy against every person in the frame.
      for (const auto & other : detections.detections) {

        if (other.results.empty()) {
          continue;
        }

        const bool other_is_person =
          other.results[0].hypothesis.class_id == "person";

        if (
          other_is_person &&
          boxes_overlap(detection.bbox, other.bbox))
        {
          overlaps_person = true;
          break;
        }
      }

      // Completely ignore this candy observation if it overlaps
      // any person. Do not calculate depth or update targets_.
      if (overlaps_person) {
        continue;
      }
    }

    TrackedTarget target;

    // Bounding-box information
    const double u = detection.bbox.center.position.x;
    const double v = detection.bbox.center.position.y;

    const float distance = depth_at(
      depth,
      static_cast<int>(std::round(u)),
      static_cast<int>(std::round(v)));

    if (!std::isfinite(distance) || distance <= 0.0f) {
      continue;
    }


    // Coordinates to camera
    double xc = ((u - cx) * distance) / fx;
    const double yc = ((v - cy) * distance) / fy;
    double zc = distance;
    if (isperson) {
      // Depth hits the front of the body/head box, about 0.2 m before its center.
      // Correct horizontally; the detection's vertical center already aims at head height.
      const double horizontal_range = std::hypot(xc, zc);
      xc += 0.2 * xc / horizontal_range;
      zc += 0.2 * zc / horizontal_range;
    }


    // Coordinates to ghost
    const double xb = zc + kCameraForward;
    const double yb = -xc + kCameraLeft;
    const double zb = -yc + kCameraUp;


    // Coordinates to world
    const double tx =
      ghost_pose.position.x +
      xb * std::cos(yaw) -
      yb * std::sin(yaw);

    const double ty =
      ghost_pose.position.y +
      xb * std::sin(yaw) +
      yb * std::cos(yaw);

    const double tz =
      ghost_pose.position.z + zb;


    // Create tracked target
    target.position.x = tx;
    target.position.y = ty;
    target.position.z = tz;

    target.is_person = isperson;
    target.last_seen = current_time;


    // Match the nearest track, retaining completion even on later detections.
    TrackedTarget * match = nullptr;
    // double best_distance = isperson ? 1.0 : 1.0;
    double best_distance = 1.0;

    for (auto & existing_target : targets_) {

      // Never match a person with candy.
      // Haunted people disappear from the simulator. A later person detection is
      // another live person, even when walking through a completed track's position.
      if (target.is_person != existing_target.is_person ||
        (isperson && existing_target.completed))
      {
        continue;
      }

      const double target_distance = std::hypot(
        existing_target.position.x - target.position.x,
        existing_target.position.y - target.position.y,
        existing_target.position.z - target.position.z);

      if (target_distance < best_distance) {
        best_distance = target_distance;
        match = &existing_target;
      }
    }

    if (match) {
      // Candy is stationary: retain its original position to avoid track drift.
      if (isperson) {
        match->position = target.position;
      }
      match->last_seen = target.last_seen;
    } else {
      targets_.push_back(target);
    }
  }

  // Negative observations only count inside the camera's view and with clear depth.
  // Transform the remembered world point back into the camera's optical frame.
  targets_.erase(std::remove_if(targets_.begin(), targets_.end(),
    [&](const TrackedTarget & target) {
      if (target.is_person || target.completed || target.last_seen >= current_time ||
        fx <= 0.0 || fy <= 0.0)
      {
        return false;
      }
      const double dx = target.position.x - ghost_pose.position.x;
      const double dy = target.position.y - ghost_pose.position.y;
      const double z = dx * std::cos(yaw) + dy * std::sin(yaw) - kCameraForward;
      const double x = dx * std::sin(yaw) - dy * std::cos(yaw) + kCameraLeft;
      const double y = ghost_pose.position.z + kCameraUp - target.position.z;
      // The simulator's camera detects targets out to 12 m.
      if (z <= 0.05 || std::hypot(x, y, z) > 12.0) {
        return false;
      }
      const double u = fx * x / z + cx;
      const double v = fy * y / z + cy;
      // Leave a pixel at the image edge for rounding and localization uncertainty.
      if (u < 1.0 || v < 1.0 || u >= depth.width - 1.0 || v >= depth.height - 1.0) {
        return false;
      }
      // A detection still counts even if its depth was unusable for a track update.
      for (const auto & detection : detections.detections) {
        if (!detection.results.empty() &&
          detection.results[0].hypothesis.class_id == "candy" &&
          std::abs(u - detection.bbox.center.position.x) <= detection.bbox.size_x / 2.0 + 1.0 &&
          std::abs(v - detection.bbox.center.position.y) <= detection.bbox.size_y / 2.0 + 1.0)
        {
          return false;
        }
      }
      const float observed_depth = depth_at(depth,
        static_cast<int>(std::round(u)), static_cast<int>(std::round(v)));
      // NaN is unknown; nearer surfaces mean the candy could be occluded.
      // Infinity means the ray is clear. Allow 10 cm for depth/pose uncertainty.
      return !std::isnan(observed_depth) && observed_depth > 0.0f &&
             observed_depth >= z - 0.1;
    }), targets_.end());

}

}  // namespace ghost_brain
