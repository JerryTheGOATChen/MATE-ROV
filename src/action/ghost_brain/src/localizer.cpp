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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>

namespace ghost_brain
{
namespace
{
constexpr double kHitHistorySeconds = 1.2;
constexpr std::size_t kMaxHits = 60;
constexpr double kMaxMatchDistance = 0.4;  // Metres: cap outliers (e.g., furniture seen at a different height)
constexpr double kSearchRadius = 0.4;      // Maximum XY offset tested at each scan
constexpr double kCorrectionGain = 0.35;   // Blend sonar match with dead reckoning
constexpr double kMinImprovement = 0.012;  // Metres of average matching error

// Extract yaw from a yaw-only orientation quaternion.
double yaw_from_pose(const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// Find the distance between a world point and the nearest occupied map cell.
// Use the *cell area*, not just its centre, for a smoother matching score.
double distance_to_map_obstacle(
  double x, double y, const nav_msgs::msg::OccupancyGrid & map)
{
  const double resolution = static_cast<double>(map.info.resolution);
  const double x0 = map.info.origin.position.x;
  const double y0 = map.info.origin.position.y;
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);

  const int col = static_cast<int>(std::floor((x - x0) / resolution));
  const int row = static_cast<int>(std::floor((y - y0) / resolution));
  const int radius_cells = static_cast<int>(std::ceil(kMaxMatchDistance / resolution)) + 1;

  double best = kMaxMatchDistance;
  for (int r = std::max(0, row - radius_cells);
       r <= std::min(height - 1, row + radius_cells); ++r)
  {
    for (int c = std::max(0, col - radius_cells);
         c <= std::min(width - 1, col + radius_cells); ++c)
    {
      const auto index = static_cast<std::size_t>(r) * map.info.width +
        static_cast<std::size_t>(c);
      if (map.data[index] <= 0) {  // Only occupied cells, not unknown/free cells
        continue;
      }

      const double cell_center_x = x0 + (static_cast<double>(c) + 0.5) * resolution;
      const double cell_center_y = y0 + (static_cast<double>(r) + 0.5) * resolution;
      const double dx = std::max(0.0, std::abs(x - cell_center_x) - resolution * 0.5);
      const double dy = std::max(0.0, std::abs(y - cell_center_y) - resolution * 0.5);
      best = std::min(best, std::hypot(dx, dy));
      if (best <= 0.0) {
        return 0.0;
      }
    }
  }
  return best;
}

}  // namespace

void Localizer::reset(const geometry_msgs::msg::Pose & start)
{
  pose_ = start;
  previous_velocity_time_ = 0.0;
  have_previous_velocity_ = false;
  initialized_ = true;
  sonar_hits_.clear();
}

void Localizer::on_velocity(const geometry_msgs::msg::TwistStamped & velocity)
{
  if (!initialized_) {
    return;
  }

  const double current_time = static_cast<double>(velocity.header.stamp.sec) +
    static_cast<double>(velocity.header.stamp.nanosec) * 1e-9;

  if (!have_previous_velocity_) {
    previous_velocity_time_ = current_time;
    have_previous_velocity_ = true;
    return;
  }

  const double dt = current_time - previous_velocity_time_;
  previous_velocity_time_ = current_time;
  if (dt <= 0.0 || dt > 0.5) {
    return;  // Ignore invalid/out-of-order time steps or big pauses
  }

  const double vx = velocity.twist.linear.x;
  const double vy = velocity.twist.linear.y;
  const double vz = velocity.twist.linear.z;
  const double yaw = yaw_from_pose(pose_);

  pose_.position.x += (vx * std::cos(yaw) - vy * std::sin(yaw)) * dt;
  pose_.position.y += (vx * std::sin(yaw) + vy * std::cos(yaw)) * dt;
  pose_.position.z += vz * dt;
}

void Localizer::on_depth(
  const sensor_msgs::msg::Image & depth, const sensor_msgs::msg::CameraInfo & info,
  const haunted_house_msgs::msg::BoxArray & obstacles)
{
  if (!initialized_ || depth.encoding != "32FC1" || depth.is_bigendian ||
    depth.width != info.width || depth.height != info.height ||
    depth.step < depth.width * sizeof(float) ||
    depth.data.size() < static_cast<std::size_t>(depth.step) * depth.height ||
    !std::isfinite(info.k[0]) || !std::isfinite(info.k[4]) ||
    info.k[0] <= 0.0 || info.k[4] <= 0.0)
  {
    return;
  }
  const double yaw = yaw_from_pose(pose_);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  // Same camera mounting offsets as Tracker: 25 cm forward, 10 cm up.
  const double ox = pose_.position.x + 0.25 * c;
  const double oy = pose_.position.y + 0.25 * s;
  std::vector<double> heights;
  for (unsigned v = depth.height * 3 / 4; v < depth.height; v += 4) {
    const double down = (static_cast<double>(v) + 0.5 - info.k[5]) / info.k[4];
    if (down < 0.15) {
      continue;
    }
    for (unsigned u = 2; u + 2 < depth.width; u += 4) {
      float range;
      std::memcpy(&range, &depth.data[v * depth.step + u * sizeof(float)], sizeof(range));
      if (!std::isfinite(range) || range <= 0.0f) {
        continue;
      }
      // If this pixel hits the floor (z=0), infer the ghost's center height.
      const double height = range * down - 0.1;
      if (height < 0.3 || height > 2.7 || std::abs(height - pose_.position.z) > 0.75) {
        continue;
      }
      const double right = (static_cast<double>(u) + 0.5 - info.k[2]) / info.k[0];
      const double dx = range * (c + right * s);
      const double dy = range * (s - right * c);
      const double oz = height + 0.1;
      bool blocked = false;
      // Check the entire hypothesized camera-to-floor segment against static geometry.
      // Furniture tops and walls must not masquerade as the floor.
      for (const auto & box : obstacles.boxes) {
        double enter = 0.0;
        double leave = 1.0;
        const auto clip = [&](double origin, double delta, double low, double high) {
          if (std::abs(delta) < 1e-9) {
            return origin >= low && origin <= high;
          }
          double a = (low - origin) / delta;
          double b = (high - origin) / delta;
          if (a > b) {
            std::swap(a, b);
          }
          enter = std::max(enter, a);
          leave = std::min(leave, b);
          return enter <= leave;
        };
        if (clip(ox, dx, box.min.x, box.max.x) &&
          clip(oy, dy, box.min.y, box.max.y) &&
          clip(oz, -oz, box.min.z, box.max.z))
        {
          blocked = true;
          break;
        }
      }
      if (!blocked) {
        heights.push_back(height);
      }
    }
  }
  if (heights.size() < 12) {
    return;  // No reliable floor view: retain vertical dead reckoning.
  }
  std::sort(heights.begin(), heights.end());
  const double median = heights[heights.size() / 2];
  const auto support = std::count_if(heights.begin(), heights.end(), [median](double height) {
    return std::abs(height - median) < 0.06;
  });
  if (support < 12 || static_cast<std::size_t>(support) * 4 < heights.size() * 3) {
    return;
  }
  pose_.position.z += std::clamp(0.25 * (median - pose_.position.z), -0.05, 0.05);
}

void Localizer::on_imu(const sensor_msgs::msg::Imu & imu)
{
  if (initialized_) {
    pose_.orientation = imu.orientation;
  }
}

void Localizer::on_sonar(
  const sensor_msgs::msg::LaserScan & scan,
  const nav_msgs::msg::OccupancyGrid & map)
{
  if (!initialized_ || map.info.resolution <= 0.0f || map.info.width == 0 ||
      map.info.height == 0 ||
      map.data.size() < static_cast<std::size_t>(map.info.width) * map.info.height ||
      scan.ranges.empty())
  {
    return;
  }

  // Standard LaserScan convention: beam angles are relative to the sonar's frame,
  // assumed aligned with the ghost's forward direction and located at its centre.
  const double yaw = yaw_from_pose(pose_);
  const double stamp = static_cast<double>(scan.header.stamp.sec) +
    static_cast<double>(scan.header.stamp.nanosec) * 1e-9;
  // A sweep message may contain only a small angular sector of valid ranges.
  // Choose samples from the VALID beams, rather than spacing them across the
  // entire array (which might contain mostly infinity/no-return readings).
  std::vector<std::size_t> valid_indices;
  valid_indices.reserve(scan.ranges.size());
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double range = static_cast<double>(scan.ranges[i]);
    if (std::isfinite(range) && range > scan.range_min &&
        range < scan.range_max - 0.01) {
      valid_indices.push_back(i);
    }
  }

  const std::size_t samples = std::min<std::size_t>(5, valid_indices.size());
  for (std::size_t j = 0; j < samples; ++j) {
    const std::size_t selected = (samples == 1) ? 0 :
      j * (valid_indices.size() - 1) / (samples - 1);
    const std::size_t i = valid_indices[selected];
    const double range = static_cast<double>(scan.ranges[i]);
    const double bearing = yaw + static_cast<double>(scan.angle_min) +
      static_cast<double>(i) * scan.angle_increment;
    sonar_hits_.push_back({
      pose_.position.x + range * std::cos(bearing),
      pose_.position.y + range * std::sin(bearing),
      stamp});
  }

  // Remove old observations: the ghost may move during the sonar sweep.
  sonar_hits_.erase(
    std::remove_if(sonar_hits_.begin(), sonar_hits_.end(), [stamp](const SonarHit & hit) {
      return stamp - hit.stamp > kHitHistorySeconds || stamp < hit.stamp;
    }), sonar_hits_.end());
  if (sonar_hits_.size() > kMaxHits) {
    sonar_hits_.erase(sonar_hits_.begin(), sonar_hits_.end() - kMaxHits);
  }
  if (sonar_hits_.size() < 6) {
    return;  // A single partial sweep usually does not constrain XY position well.
  }

  // Score an XY translation of the accumulated endpoints against the 2D map.
  // Truncated errors make individual mismatches less influential.
  struct Score
  {
    double cost;
    int matches;
  };
  const auto score_shift = [&](double dx, double dy) -> Score {
    double cost = 0.0;
    int matches = 0;
    for (const auto & hit : sonar_hits_) {
      const double error = distance_to_map_obstacle(hit.x + dx, hit.y + dy, map);
      cost += error;
      if (error < 0.20) {
        ++matches;
      }
    }
    cost /= static_cast<double>(sonar_hits_.size());
    cost += 0.03 * std::hypot(dx, dy);  // Prefer smaller corrections when scores are similar.
    return {cost, matches};
  };

  const Score baseline = score_shift(0.0, 0.0);
  Score best = baseline;
  double best_dx = 0.0;
  double best_dy = 0.0;

  // Coarse search then local refinement. Two stages are cheaper than a fine
  // exhaustive 2D grid, while still allowing sub-cell correction.
  const auto search_near = [&](double cx, double cy, double step, int steps) {
    for (int iy = -steps; iy <= steps; ++iy) {
      for (int ix = -steps; ix <= steps; ++ix) {
        const double dx = cx + static_cast<double>(ix) * step;
        const double dy = cy + static_cast<double>(iy) * step;
        if (std::abs(dx) > kSearchRadius || std::abs(dy) > kSearchRadius) {
          continue;
        }
        const Score candidate = score_shift(dx, dy);
        if (candidate.cost < best.cost) {
          best = candidate;
          best_dx = dx;
          best_dy = dy;
        }
      }
    }
  };

  search_near(0.0, 0.0, 0.1, 4);
  search_near(best_dx, best_dy, 0.025, 4);

  // Require sufficient agreement with mapped obstacles and meaningful improvement
  // before changing the pose; avoid chasing sonar artifacts/noisy partial scans.
  const int min_matches = std::max(3, static_cast<int>(sonar_hits_.size() / 4));
  if (best.matches < min_matches || baseline.cost - best.cost < kMinImprovement) {
    return;
  }

  const double dx = kCorrectionGain * best_dx;
  const double dy = kCorrectionGain * best_dy;
  pose_.position.x += dx;
  pose_.position.y += dy;

  // Existing history is expressed in the old estimated world frame. Shift it
  // consistently so later scan matching does not apply the same correction twice.
  for (auto & hit : sonar_hits_) {
    hit.x += dx;
    hit.y += dy;
  }
}

}  // namespace ghost_brain