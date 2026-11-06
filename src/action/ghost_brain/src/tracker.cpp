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
#include <cmath>
#include <cstring>
#include <limits>
#include <string>

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

namespace
{

/// Targets closer together than this (m) count as the same one
constexpr double kMatchRadius = 0.8;

/// Candy never moves, so sightings of it that disagree by more than the noise are the ghost's own
/// position error: be generous, or every drift makes a second copy
constexpr double kCandyMatchRadius = 1.1;

/// Only believe candy seen closer than this (m)
constexpr double kMaxCandyRange = 6.0;


/// A person not seen for this long (s) has walked away: forget them rather than chase where they were
constexpr double kPersonForgetSeconds = 8.0;

/// Candy never moves, so a new sighting only moves a known one this fraction of the way: it averages
/// out the noise (especially while the ghost is turning) instead of jumping about
constexpr double kCandyBlend = 0.3;

/// People's boxes are around their head, and the depth is to the front of it: about this far
/// (m) nearer than its center
constexpr double kHeadDepthOffset = 0.2;

/// A person's head is never closer than this (m) to a wall or furniture
constexpr double kHeadClearance = 0.25;

/// A target this close to the ghost has been collected (the catch radius is 0.5 m), so forget it
constexpr double kCollectRadius = 0.5;

/// How much of a new velocity estimate to mix in (the rest is the old one), to smooth out noise
constexpr double kVelocityBlend = 0.4;

/// Work out a person's velocity over at least this long (s), so a little position noise does not
/// turn into a huge speed, and never believe a speed above this (m/s): people walk
constexpr double kVelocityBaseline = 0.6;
constexpr double kMaxPersonSpeed = 1.0;

/// The nearest depth inside a box of pixels (center u, v; size w, h), ignoring pixels with no reading
/// (infinite or NaN). Returns infinity if there is none.
float nearest_depth_in_box(
  const sensor_msgs::msg::Image & depth, double center_u, double center_v, double width, double height)
{
  const int u0 = static_cast<int>(std::floor(center_u - width / 2.0));
  const int u1 = static_cast<int>(std::ceil(center_u + width / 2.0));
  const int v0 = static_cast<int>(std::floor(center_v - height / 2.0));
  const int v1 = static_cast<int>(std::ceil(center_v + height / 2.0));
  float best = std::numeric_limits<float>::infinity();
  for (int v = v0; v <= v1; ++v) {
    for (int u = u0; u <= u1; ++u) {
      const float d = depth_at(depth, u, v);
      if (std::isfinite(d) && d > 0.0f) {
        best = std::min(best, d);
      }
    }
  }
  return best;
}

/// The ghost's heading (rotation about z) from its orientation quaternion, in radians
double yaw_of(const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

/// The point a pixel (u, v) at this depth is, in the camera's optical frame
/// (x right, y down, z forward). Use the pinhole model with camera_info.k = {fx, 0, cx, 0, fy, cy, ...}
geometry_msgs::msg::Point pixel_to_camera(
  double u, double v, double depth, const sensor_msgs::msg::CameraInfo & camera_info)
{
  geometry_msgs::msg::Point camera;
  const double fx = camera_info.k[0];
  const double cx = camera_info.k[2];
  const double fy = camera_info.k[4];
  const double cy = camera_info.k[5];

  camera.x = (u - cx) * depth / fx;
  camera.y = (v - cy) * depth / fy;
  camera.z = depth;

  return camera;
}

/// The same point in the ghost's own frame (x forward, y left, z up). That is the camera's
/// axes swapped around, then shifted by kCameraForward / kCameraLeft / kCameraUp
geometry_msgs::msg::Point camera_to_ghost(const geometry_msgs::msg::Point & p)
{
  geometry_msgs::msg::Point ghost;

  ghost.x = p.z + kCameraForward;
  ghost.y = -p.x + kCameraLeft;
  ghost.z = -p.y + kCameraUp;


  return ghost;
}

/// The same point in the map: rotate by the ghost's yaw (yaw_of), then add its position
geometry_msgs::msg::Point ghost_to_map(
  const geometry_msgs::msg::Point & p, const geometry_msgs::msg::Pose & ghost_pose)
{
  geometry_msgs::msg::Point map;
  const double yaw = yaw_of(ghost_pose);

  map.x = std::cos(yaw) * p.x - std::sin(yaw) * p.y + ghost_pose.position.x;
  map.y = std::sin(yaw) * p.x + std::cos(yaw) * p.y + ghost_pose.position.y;
  map.z = p.z + ghost_pose.position.z;

  return map;
}

double distance_between(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  return std::hypot(a.x - b.x, a.y - b.y, a.z - b.z);
}

/// Whether the point is within margin (m) of any obstacle box
bool inside_obstacle(
  const haunted_house_msgs::msg::BoxArray & obstacles, const geometry_msgs::msg::Point & p, double margin)
{
  for (const auto & box : obstacles.boxes) {
    const double dx = std::max({box.min.x - p.x, 0.0, p.x - box.max.x});
    const double dy = std::max({box.min.y - p.y, 0.0, p.y - box.max.y});
    const double dz = std::max({box.min.z - p.z, 0.0, p.z - box.max.z});
    if (std::hypot(dx, dy, dz) < margin) {
      return true;
    }
  }
  return false;
}

}  // namespace

void Tracker::reset()
{
  targets_.clear();
}

void Tracker::forget_nearest(bool is_person, const geometry_msgs::msg::Point & near, double radius)
{
  auto best = targets_.end();
  double best_gap = radius;
  for (auto it = targets_.begin(); it != targets_.end(); ++it) {
    const double gap = distance_between(it->position, near);
    if (it->is_person == is_person && gap < best_gap) {
      best = it;
      best_gap = gap;
    }
  }
  if (best != targets_.end()) {
    targets_.erase(best);
  }
}

void Tracker::on_detections(
  const vision_msgs::msg::Detection2DArray & detections,
  const sensor_msgs::msg::Image & depth,
  const sensor_msgs::msg::CameraInfo & camera_info,
  const geometry_msgs::msg::Pose & ghost_pose,
  const haunted_house_msgs::msg::BoxArray * obstacles)
{
  for (const auto & detection : detections.detections) {
    if (detection.results.empty()) {
      continue;
    }
    const bool is_person = detection.results[0].hypothesis.class_id == "person";

    // The detection's box in pixels
    const double u = detection.bbox.center.position.x;
    const double v = detection.bbox.center.position.y;
    // Use the depth right at the box's center. The nearest depth anywhere in the box often catches
    // the edge of something in front (a door frame, a table), which puts the target on that instead.
    // Only if the center has no reading, fall back to the nearest one
    float d = depth_at(depth, static_cast<int>(std::round(u)), static_cast<int>(std::round(v)));
    if (!std::isfinite(d) || d <= 0.0f) {
      d = nearest_depth_in_box(depth, u, v, detection.bbox.size_x, detection.bbox.size_y);
    }
    if (!std::isfinite(d)) {
      continue;  // nothing measured there
    }
    if (!is_person && d > kMaxCandyRange) {
      continue;  // a far candy is a pixel or two: its depth often comes from the wall behind it
    }
    // Depth is to the front of a head: add back the distance to its center
    const double depth_to_center = is_person ? d + kHeadDepthOffset : d;

    const auto in_camera = pixel_to_camera(u, v, depth_to_center, camera_info);
    const auto in_ghost = camera_to_ghost(in_camera);
    const auto in_map = ghost_to_map(in_ghost, ghost_pose);

    // A head cannot be inside a wall or a shelf: a person placed there was seen past a door frame
    // or some furniture, and the depth came from that instead. Ignore the sighting
    if (is_person && obstacles && inside_obstacle(*obstacles, in_map, kHeadClearance)) {
      continue;
    }
    const rclcpp::Time now(depth.header.stamp);

    // Is this one we already know? Take the nearest target of the same kind within kMatchRadius
    TrackedTarget * match = nullptr;
    const double best = is_person ? kMatchRadius : kCandyMatchRadius;
    double best_gap = 1e9;
    for (auto & target : targets_) {
      if (target.is_person != is_person) {
        continue;
      }
      double gap = distance_between(target.position, in_map);
      double limit = best;
      if (is_person) {
        // A person walks on while out of sight: look for them where they should have got to, and
        // allow more slack the longer it has been
        const double age = std::min((now - target.last_seen).seconds(), 4.0);
        geometry_msgs::msg::Point expected = target.position;
        expected.x += target.velocity.x * age;
        expected.y += target.velocity.y * age;
        gap = std::min(gap, distance_between(expected, in_map));
        limit = kMatchRadius + 0.6 * age;
      }
      if (gap < limit && (match == nullptr || gap < best_gap)) {
        match = &target;
        best_gap = gap;
      }
    }

    if (match == nullptr) {
      TrackedTarget fresh;
      fresh.position = in_map;
      fresh.is_person = is_person;
      fresh.last_seen = now;
      fresh.anchor = in_map;
      fresh.anchor_time = now;
      targets_.push_back(fresh);
    } else {
      // People walk: estimate their velocity from how far they moved over a good while
      const double dt = (now - match->anchor_time).seconds();
      if (is_person && dt >= kVelocityBaseline) {
        double vx = (in_map.x - match->anchor.x) / dt;
        double vy = (in_map.y - match->anchor.y) / dt;
        const double speed = std::hypot(vx, vy);
        if (speed > kMaxPersonSpeed) {
          vx *= kMaxPersonSpeed / speed;
          vy *= kMaxPersonSpeed / speed;
        }
        match->velocity.x += kVelocityBlend * (vx - match->velocity.x);
        match->velocity.y += kVelocityBlend * (vy - match->velocity.y);
        match->anchor = in_map;
        match->anchor_time = now;
      }
      if (is_person) {
        match->position = in_map;
      } else {
        match->position.x += kCandyBlend * (in_map.x - match->position.x);
        match->position.y += kCandyBlend * (in_map.y - match->position.y);
        match->position.z += kCandyBlend * (in_map.z - match->position.z);
      }
      match->last_seen = now;
    }
  }

  // People who have not been seen for a while have walked on: forget them
  {
    const rclcpp::Time latest(depth.header.stamp);
    targets_.erase(
      std::remove_if(
        targets_.begin(), targets_.end(),
        [&](const TrackedTarget & target) {
          return target.is_person && (latest - target.last_seen).seconds() > kPersonForgetSeconds;
        }),
      targets_.end());
  }

  // Anything the ghost is on top of has been collected: forget it
  targets_.erase(
    std::remove_if(
      targets_.begin(), targets_.end(),
      [&](const TrackedTarget & target) {
        return distance_between(target.position, ghost_pose.position) < kCollectRadius;
      }),
    targets_.end());
}

}  // namespace ghost_brain
