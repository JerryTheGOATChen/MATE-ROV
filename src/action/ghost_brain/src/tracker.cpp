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
#include "ghost_brain/planner.hpp"

#include <cstring>
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Vector3.hpp"

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
  next_id_ = 1;
  frame_ = 0;
  random_target_.reset();
  skipped_position_.reset();
  candy_goal_.reset();
}

void Tracker::skip_target(const TrackedTarget & target)
{
  skipped_position_ = target.position;
  skip_until_ = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  remove(target.id);
  random_target_.reset();
}

void Tracker::remove(uint64_t id)
{
  if (candy_goal_ && candy_goal_->id == id) {
    candy_goal_.reset();
  }
  if (id == 0) {
    random_target_.reset();
    return;
  }
  targets_.erase(std::remove_if(targets_.begin(), targets_.end(),
    [id](const TrackedTarget & target) { return target.id == id; }), targets_.end());
}

std::optional<TrackedTarget> Tracker::choose_target(
  const geometry_msgs::msg::Point & here, const nav_msgs::msg::OccupancyGrid & map)
{
  // Candy inside the obstacle buffer is not a valid destination, even if seen again.
  targets_.erase(std::remove_if(targets_.begin(), targets_.end(), [&](const TrackedTarget & target) {
    return !target.is_person && is_occupied_with_clearance(map, target.position.x, target.position.y);
  }), targets_.end());
  auto distance = [&](const geometry_msgs::msg::Point & point) {
    return std::hypot(point.x - here.x, point.y - here.y, point.z - here.z);
  };
  if (skipped_position_ && std::chrono::steady_clock::now() >= skip_until_) {
    skipped_position_.reset();
  }
  auto skipped = [&](const geometry_msgs::msg::Point & point) {
    return skipped_position_ && std::hypot(
      point.x - skipped_position_->x, point.y - skipped_position_->y,
      point.z - skipped_position_->z) <= 0.5;
  };
  std::optional<TrackedTarget> person;
  std::optional<TrackedTarget> candy;
  for (const auto & target : targets_) {
    if (skipped(target.position)) {
      continue;
    }
    if (target.is_person && target.seen_frame != frame_) {
      continue;
    }
    auto & closest = target.is_person ? person : candy;
    if (!closest || distance(target.position) < distance(closest->position)) {
      closest = target;
    }
  }
  if (person) {
    // After a person, select the nearest candy anew instead of resuming an old one.
    candy_goal_.reset();
    random_target_.reset();
    return person;
  }
  if (candy_goal_ && (skipped(candy_goal_->position) ||
      is_occupied_with_clearance(map, candy_goal_->position.x, candy_goal_->position.y) ||
      std::none_of(targets_.begin(), targets_.end(), [&](const TrackedTarget & target) {
        return target.id == candy_goal_->id;
      }))) {
    candy_goal_.reset();
  }
  if (!candy_goal_ && candy) {
    candy_goal_ = candy;  // Keep this position fixed despite new sightings or changing distances.
  }
  if (candy_goal_) {
    random_target_.reset();
    return candy_goal_;
  }

  return choose_random_target(here, map);
}

std::optional<TrackedTarget> Tracker::choose_random_target(
  const geometry_msgs::msg::Point & here, const nav_msgs::msg::OccupancyGrid & map)
{
  if (!std::isfinite(here.z) || is_occupied(map, here.x, here.y)) {
    random_target_.reset();
    return std::nullopt;
  }
  if (random_target_ && (std::hypot(
      here.x - random_target_->position.x, here.y - random_target_->position.y,
      here.z - random_target_->position.z) <= 0.2 ||
      is_occupied_with_clearance(map, random_target_->position.x, random_target_->position.y))) {
    random_target_.reset();
  }
  if (random_target_) {
    return random_target_;
  }
  if (skipped_position_ && std::chrono::steady_clock::now() >= skip_until_) {
    skipped_position_.reset();
  }

  const auto navigation = inflate_map(map);
  const int width = static_cast<int>(navigation.info.width);
  const int height = static_cast<int>(navigation.info.height);
  const double resolution = navigation.info.resolution;
  const int start_col = static_cast<int>(std::floor((here.x - map.info.origin.position.x) / resolution));
  const int start_row = static_cast<int>(std::floor((here.y - map.info.origin.position.y) / resolution));
  const size_t start = start_row * width + start_col;

  // Flood-fill using the same eight-neighbor and no-corner-cutting rules as A*.
  // Only reachable, clearance-safe cells can become dummy candy goals.
  std::queue<size_t> pending;
  std::vector<bool> seen(navigation.data.size(), false);
  std::vector<size_t> candidates;
  pending.push(start);
  seen[start] = true;
  const int directions[8][2] = {
    {-1, -1}, {0, -1}, {1, -1}, {-1, 0}, {1, 0}, {-1, 1}, {0, 1}, {1, 1}};
  while (!pending.empty()) {
    const auto cell = pending.front();
    pending.pop();
    const int col = static_cast<int>(cell % width);
    const int row = static_cast<int>(cell / width);
    const double x = map.info.origin.position.x + (col + 0.5) * resolution;
    const double y = map.info.origin.position.y + (row + 0.5) * resolution;
    const double z = std::max(0.3, here.z);
    if (navigation.data[cell] == 0 && (!skipped_position_ || std::hypot(
        x - skipped_position_->x, y - skipped_position_->y, z - skipped_position_->z) > 0.5)) {
      candidates.push_back(cell);
    }
    for (const auto & direction : directions) {
      const int nx = col + direction[0];
      const int ny = row + direction[1];
      if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
        continue;
      }
      const size_t next = ny * width + nx;
      if (seen[next] || !is_navigation_step_valid(map, navigation, col, row, nx, ny)) {
        continue;
      }
      seen[next] = true;
      pending.push(next);
    }
  }
  if (candidates.empty()) {
    return std::nullopt;
  }
  std::uniform_int_distribution<size_t> pick(0, candidates.size() - 1);
  const auto cell = candidates[pick(random_)];
  TrackedTarget dummy;
  dummy.is_temporary = true;
  dummy.position.x = map.info.origin.position.x + (cell % width + 0.5) * resolution;
  dummy.position.y = map.info.origin.position.y + (cell / width + 0.5) * resolution;
  dummy.position.z = std::max(0.3, here.z);
  random_target_ = dummy;
  return random_target_;
}

void Tracker::on_detections(
  const vision_msgs::msg::Detection2DArray & detections,
  const sensor_msgs::msg::Image & depth,
  const sensor_msgs::msg::CameraInfo & camera_info,
  const geometry_msgs::msg::Pose & ghost_pose)
{
  const double fx = camera_info.k[0];
  const double fy = camera_info.k[4];
  const double cx = camera_info.k[2];
  const double cy = camera_info.k[5];
  if (!std::isfinite(fx) || !std::isfinite(fy) || fx <= 0.0 || fy <= 0.0 ||
      depth.encoding != "32FC1" || depth.step < depth.width * sizeof(float) ||
      depth.data.size() < static_cast<size_t>(depth.step) * depth.height) {
    return;
  }

  ++frame_;
  std::vector<bool> matched(targets_.size(), false);
  const auto & q = ghost_pose.orientation;
  const tf2::Quaternion orientation(q.x, q.y, q.z, q.w);
  for (const auto & detection : detections.detections) {
    if (detection.results.empty()) {
      continue;
    }
    const auto & label = detection.results[0].hypothesis.class_id;
    if (label != "person" && label != "candy") {
      continue;
    }
    const bool is_person = label == "person";
    const double u = detection.bbox.center.position.x;
    const double v = detection.bbox.center.position.y;
    if (!std::isfinite(u) || !std::isfinite(v) ||
        u < 0 || v < 0 || u >= depth.width || v >= depth.height) {
      continue;
    }
    const float d = depth_at(depth, static_cast<int>(u), static_cast<int>(v));
    if (!std::isfinite(d) || d <= 0.0f) {
      continue;
    }

    // Optical frame (right, down, forward) -> ghost frame (forward, left, up).
    tf2::Vector3 relative(d, -(u - cx) * d / fx, -(v - cy) * d / fy);
    if (is_person) {
      relative += relative.normalized() * 0.2;  // Front of head -> estimated center.
    }
    relative += tf2::Vector3(kCameraForward, kCameraLeft, kCameraUp);
    const auto rotated = tf2::quatRotate(orientation, relative);
    geometry_msgs::msg::Point position;
    position.x = ghost_pose.position.x + rotated.x();
    position.y = ghost_pose.position.y + rotated.y();
    position.z = std::max(0.3, ghost_pose.position.z + rotated.z());

    size_t best = targets_.size();
    double nearest = 0.5;
    for (size_t i = 0; i < matched.size(); ++i) {
      const auto & target = targets_[i];
      const double d = std::hypot(position.x - target.position.x,
        position.y - target.position.y, position.z - target.position.z);
      if (!matched[i] && target.is_person == is_person && d < nearest) {
        best = i;
        nearest = d;
      }
    }
    const rclcpp::Time stamp(detections.header.stamp);
    if (best == targets_.size()) {
      targets_.push_back({position, is_person, stamp, next_id_++, frame_});
    } else {
      targets_[best].position = position;
      targets_[best].last_seen = stamp;
      targets_[best].seen_frame = frame_;
      matched[best] = true;
    }
  }
  targets_.erase(std::remove_if(targets_.begin(), targets_.end(), [this](const TrackedTarget & target) {
    return target.is_person && target.seen_frame != frame_;
  }), targets_.end());

}

}  // namespace ghost_brain
