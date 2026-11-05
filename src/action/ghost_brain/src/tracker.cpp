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
#include <algorithm>
#include <unordered_set>

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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

void Tracker::on_collected(bool is_person, const geometry_msgs::msg::Point& ghost_position) {
  // The ghost just caught something of this kind, so it's the closest one to the ghost
  auto closest = targets_.end();
  double closest_dist = std::numeric_limits<double>::infinity();
  for (auto it = targets_.begin(); it != targets_.end(); ++it) {
    if (it->is_person != is_person) continue;
    const double dist = std::hypot(
      it->position.x - ghost_position.x, it->position.y - ghost_position.y, it->position.z - ghost_position.z);
    if (dist < closest_dist) {
      closest = it;
      closest_dist = dist;
    }
  }
  if (closest != targets_.end()) {
    targets_.erase(closest);
  }
}

void Tracker::on_detections(
  const vision_msgs::msg::Detection2DArray & detections,
  const sensor_msgs::msg::Image & depth,
  const sensor_msgs::msg::CameraInfo & camera_info,
  const geometry_msgs::msg::Pose & ghost_pose)
{
  // TODO(hackathon): work out where each detection is in the map, and keep track of them in targets_
  const double shrink_factor = 1.0;
  rclcpp::Time current_time(detections.header.stamp);

  std::unordered_set<TrackedTarget*> matched_targets;
  // Adding to a vector can move everything in it, which would break the pointers above. With the
  // room set aside up front, nothing moves
  targets_.reserve(targets_.size() + detections.detections.size());

  for (const auto& detection : detections.detections){
    if (detection.results.empty()) {
      continue;
    }
    const auto& bbox = detection.bbox;

    //Bounds u is x, v is y
    int u_min = std::floor(bbox.center.position.x - (bbox.size_x * shrink_factor) / 2.0);
    int u_max = std::floor(bbox.center.position.x + (bbox.size_x * shrink_factor) / 2.0);
    int v_min = std::floor(bbox.center.position.y - (bbox.size_y * shrink_factor) / 2.0);
    int v_max = std::floor(bbox.center.position.y + (bbox.size_y * shrink_factor) / 2.0);

    u_min = std::max(0, u_min);
    u_max = std::min(static_cast<int>(depth.width) - 1, u_max);
    v_min = std::max(0, v_min);
    v_max = std::min(static_cast<int>(depth.height) - 1, v_max);

    //Safe guard
    if (u_max < u_min || v_max < v_min) continue;

    //Search for minimum distance
    float min_depth = std::numeric_limits<float>::infinity();
    for (int u = u_min; u <= u_max; ++u){
      for (int v = v_min; v <= v_max; ++v){
        auto d = depth_at(depth, u, v);
        if (d < min_depth && std::isfinite(d)) min_depth = d;
      }
    }

    if(min_depth == std::numeric_limits<float>::infinity()) continue;

    const bool is_person = detection.results[0].hypothesis.class_id == "person";

    // Candy boxes are centre
    float target_depth = min_depth;
    if (!is_person) {
      const float center_depth = depth_at(
        depth, static_cast<int>(std::floor(bbox.center.position.x)), static_cast<int>(std::floor(bbox.center.position.y)));
      if (std::isfinite(center_depth)) target_depth = center_depth;
    }

    //Optical Frame
    const double fx = camera_info.k[0];
    const double cx = camera_info.k[2];
    const double fy = camera_info.k[4];
    const double cy = camera_info.k[5];
    const double u_center = detection.bbox.center.position.x;
    const double v_center = detection.bbox.center.position.y;

    double cam_x = (u_center - cx) * target_depth / fx;
    double cam_y = (v_center - cy) * target_depth / fy;
    double cam_z = target_depth;

    //Depth hits the front of a person's body, about 0.2 m before their head's center
    const double range = std::hypot(cam_x, cam_y, cam_z);
    if (is_person && range > 0.0) {
      const double scale = (range + 0.2) / range;
      cam_x *= scale;
      cam_y *= scale;
      cam_z *= scale;
    }

    //Optical Frame -> Ghost Frame
    const double forward = kCameraForward + cam_z;
    const double left = kCameraLeft - cam_x ;
    const double up = kCameraUp - cam_y ;

    //Ghost Frame -> Map
    const double yaw = tf2::getYaw(ghost_pose.orientation);
    const double map_x = ghost_pose.position.x + std::cos(yaw) * forward - std::sin(yaw) * left;
    const double map_y = ghost_pose.position.y + std::sin(yaw) * forward + std::cos(yaw) * left;
    const double map_z = ghost_pose.position.z + up;

    //Finding best point per object
    TrackedTarget* best = nullptr;
    double best_dist = std::numeric_limits<double>::infinity();

    for (auto& target : targets_){
      if (target.is_person != is_person) continue;
      if (matched_targets.count(&target)) continue;
      double dt_target = (current_time - target.last_seen).seconds();
      // A noisy velocity gets less believable with time, so only predict a little way ahead
      const double dt_predict = std::clamp(dt_target, 0.0, 2.0);
      double pred_x = target.position.x + target.velocity.linear.x * dt_predict;
      double pred_y = target.position.y + target.velocity.linear.y * dt_predict;
      double pred_z = target.position.z + target.velocity.linear.z * dt_predict;

      //Gate progressively throws out older predictions
      // Gate grows as time increases
      // Candy doesn't move, so its gate stays put
      double base_gate = is_person ? 1.0 : 0.7;
      double expanding_gate = is_person ? base_gate + (0.15 * std::max(0.0, dt_target)) : base_gate;

      double dx = pred_x - map_x;
      double dy = pred_y - map_y;
      double dz = pred_z - map_z;

      const double dist = std::hypot(dx, dy, dz);

      if (dist < best_dist && dist < expanding_gate) { // Distance
        best = &target;
        best_dist = dist;
      }

    }

    //Adding new targets or updating target
    const double w = 1.0 / (range * range);
    if (best != nullptr) {
      matched_targets.insert(best);
      best->miss_count = 0;     //Counting for how many times a object isn't in view
      if (is_person) {
        // People move, so the latest sighting is the best one
        const double dt = (current_time - best->last_seen).seconds();
        if (dt > 0.001) { // Prevent division by zero or negative dt
          // Raw instantaneous velocity in map frame
          const double alpha = settings_.alpha;
          const double beta = settings_.beta;
          
          //Prediction of next pose
          double pred_x = best->position.x + best->velocity.linear.x * dt;
          double pred_y = best->position.y + best->velocity.linear.y * dt;
          double pred_z = best->position.z + best->velocity.linear.z * dt;

          //Difference between new reading and our prediction
          double res_x = map_x - pred_x;
          double res_y = map_y - pred_y;
          double res_z = map_z - pred_z;

          //Update the position
          best->position.x = pred_x + alpha * res_x;
          best->position.y = pred_y + alpha * res_y;
          best->position.z = pred_z + alpha * res_z;

          //Update velocity
          best->velocity.linear.x += (beta / dt) * res_x;
          best->velocity.linear.y += (beta / dt) * res_y;
          best->velocity.linear.z += (beta / dt) * res_z;

          // People walk at 0.6 m/s at most, so anything faster is noise
          const double speed = std::hypot(
            best->velocity.linear.x, best->velocity.linear.y, best->velocity.linear.z);
          if (speed > 0.6) {
            best->velocity.linear.x *= 0.6 / speed;
            best->velocity.linear.y *= 0.6 / speed;
            best->velocity.linear.z *= 0.6 / speed;
          }
        } else {
          best->position.x = map_x;
          best->position.y = map_y;
          best->position.z = map_z;
        }
      } else {
        // Candy stays put, so average the sightings, trusting close ones more
        const double total = best->weighted_Sum + w;
        best->position.x = (best->weighted_Sum * best->position.x + w * map_x) / total;
        best->position.y = (best->weighted_Sum * best->position.y + w * map_y) / total;
        best->position.z = (best->weighted_Sum * best->position.z + w * map_z) / total;
        best->weighted_Sum = total;
      }
      best->last_seen = rclcpp::Time(detections.header.stamp);
    } else {
      //Target object
      TrackedTarget tracked_target;
      tracked_target.position.x = map_x;
      tracked_target.position.y = map_y;
      tracked_target.position.z = map_z;
      tracked_target.velocity.linear.x = 0.0;
      tracked_target.velocity.linear.y = 0.0;
      tracked_target.velocity.linear.z = 0.0;
      tracked_target.is_person = is_person;
      tracked_target.last_seen = current_time;
      tracked_target.weighted_Sum = w;
      tracked_target.miss_count = 0;
      targets_.push_back(tracked_target);
      // Just seen, so this can't count as a miss, or take a second detection in this frame
      matched_targets.insert(&targets_.back());
    }
  }

  const double fx = camera_info.k[0];
  const double cx = camera_info.k[2];
  const double fy = camera_info.k[4];
  const double cy = camera_info.k[5];
  const double yaw = tf2::getYaw(ghost_pose.orientation);

  for (auto& target : targets_) {
    // Check if this target was NOT matched in this frame
    if (!matched_targets.count(&target)) {
      // Check if we should be able to see it
      double dx = target.position.x - ghost_pose.position.x;
      double dy = target.position.y - ghost_pose.position.y;
      double dist_to_ghost = std::hypot(dx, dy);

      // Map -> Ghost Frame -> Optical Frame, the reverse of placing a detection
      const double forward = std::cos(yaw) * dx + std::sin(yaw) * dy;
      const double left = -std::sin(yaw) * dx + std::cos(yaw) * dy;
      const double up = target.position.z - ghost_pose.position.z;
      const double cam_x = -(left - kCameraLeft);
      const double cam_y = -(up - kCameraUp);
      const double cam_z = forward - kCameraForward;

      // Behind the camera, it can't be seen
      if (cam_z <= 0.0) continue;

      // Stay a little inside the edges of the image, in case the estimate is a bit off
      const double margin = 5.0;
      const double u = fx * cam_x / cam_z + cx;
      const double v = fy * cam_y / cam_z + cy;
      const bool in_image = u >= margin && u < depth.width - margin && v >= margin && v < depth.height - margin;
      if (!in_image) continue;

      // Something nearer is in the way (a person's depth is up to 0.2 m short, so allow for it)
      if (depth_at(depth, static_cast<int>(u), static_cast<int>(v)) < cam_z - 0.3) continue;

      // Assuming camera sees up to 5 meters forward and within sensor
      if (dist_to_ghost < 5.0) {
        target.miss_count++;
      }
    }
  }

  // Remove targets that missed too many frames or people unseen for > 10 seconds
  targets_.erase(
      std::remove_if(targets_.begin(), targets_.end(), [&](const TrackedTarget& t) {
          double total_unseen = (current_time - t.last_seen).seconds();
          bool forgotten_person = t.is_person && (total_unseen > 10.0);
          bool max_misses_exceeded = t.miss_count >= 5; 
          const double arrived_dist = t.is_person ? 1.0 : 0.6;
          bool nothing_there = total_unseen > 2.0 &&
            std::hypot(t.position.x - ghost_pose.position.x, t.position.y - ghost_pose.position.y) < arrived_dist;
          return forgotten_person || max_misses_exceeded || nothing_there;
      }),
      targets_.end()
  );
}

}  // namespace ghost_brain
