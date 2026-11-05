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
#include <array>
#include <cmath>

#include "ghost_brain/tracker.hpp"  // depth_at and kCameraUp
#include "rclcpp/logging.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace ghost_brain
{
namespace
{

// Sonar matching (lesson 11)
// a beam that lands further than this from a wall hit something else (a person), so skip it
constexpr double kInlierGate = 0.3;
// past this, a beam only counts partly (Huber), so a few bad ones can't drag the pose around
constexpr double kHuberWidth = 0.1;
// need at least this many good beams in a turn to trust it
constexpr int kMinInliers = 60;
// only move this much of the way each turn, so one bad turn can't throw it off
constexpr double kCorrectionGain = 0.4;
// keeps it from sliding along a corridor, where nothing says how far along you are
constexpr double kDamping = 1.0;
// how far the distance field looks for a wall. Anything further is just "far"
constexpr double kFieldReach = 1.0;
// how much pose history to keep, in seconds. A turn is 1 s
constexpr double kHistoryLength = 3.0;

// Height from the floor in the depth image
// only pixels looking at least this far down (per meter of depth, about 6 degrees). Flatter ones hit
// the floor so far away it's out of range anyway
constexpr double kMinDown = 0.1;
// heights get sorted into bins this big (m). The floor all lands in the same one or two
constexpr double kHeightBinSize = 0.02;
constexpr double kLowestHeight = -0.5;
constexpr double kHighestHeight = 3.5;
// look down each column this many pixels at a time
constexpr int kRunRows = 10;
// a flat run's heights barely change down the column. A wall's change by its depth per unit of "down",
// so this lets through only walls closer than 15 cm, which say a height way below anything real
constexpr double kMaxSlope = 0.15;
// and no pixel in a flat run is further than this from the run's height (the noise is about 1 cm)
constexpr double kMaxFlatError = 0.04;
// the floor has to show up in at least this many flat runs
constexpr int kMinFloorRuns = 30;
// further than this from where we think we are: that wasn't the floor (the lowest furniture is 0.6 m)
constexpr double kHeightGate = 0.25;
// the floor's very accurate (1 cm), so move most of the way. 10 frames a second
constexpr double kHeightGain = 0.3;

double wrap_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

void Localizer::reset(const geometry_msgs::msg::Pose & start)
{
  pose_ = start;
  // Forget the last game's stamp, so the first dt isn't measured from it
  has_prev_time_ = false;
  x_ = start.position.x;
  y_ = start.position.y;
  z_ = start.position.z;
  // The bias changes every game, so last game's is no use
  state_ = LocalizerState::RUNNING;
  vel_bias_x_ = 0.0;
  vel_bias_y_ = 0.0;
  vel_bias_z_ = 0.0;
  // new game, new history, and half a turn from last game is no use
  history_.clear();
  endpoints_.clear();
  turn_angle_ = 0.0;
}

//Calibration to destroy angle and velocity noise and errors
void Localizer::start_calibration()
{
  state_ = LocalizerState::CALIBRATING;
  has_calibration_start_ = false;
  calibration_count_ = 0;
  sum_vx_ = 0.0;
  sum_vy_ = 0.0;
  sum_vz_ = 0.0;
}

void Localizer::finish_calibration()
{
  if (state_ != LocalizerState::CALIBRATING) {
    return;
  }
  state_ = LocalizerState::RUNNING;
  // The ghost was frozen, so whatever the average reading is, it's the bias
  if (calibration_count_ > 0) {
    vel_bias_x_ = sum_vx_ / calibration_count_;
    vel_bias_y_ = sum_vy_ / calibration_count_;
    vel_bias_z_ = sum_vz_ / calibration_count_;
  }
}

void Localizer::on_velocity(const geometry_msgs::msg::TwistStamped & velocity)
{
  rclcpp::Time current_time = velocity.header.stamp;

  // The first message after a reset has nothing to measure dt from
  if (!has_prev_time_) {
    prev_time_ = current_time;
    has_prev_time_ = true;
    return;
  }

  // Finding dt
  const double dt = (current_time - prev_time_).seconds();

  // Remember this stamp before any early return, so the next dt is measured from here.
  // Returning first would compare every later message to the same old stamp, and skip them all
  prev_time_ = current_time;

  //Handles dt, throws away <0, or big dt vals
  if (dt <= 0.0) {
    if (dt < 0.0) {
      RCLCPP_WARN(rclcpp::get_logger("localizer"), "Negative dt detected (%f s). Skipping it.", dt);
    } else {
      RCLCPP_DEBUG(rclcpp::get_logger("localizer"), "Zero dt detected. Ignoring duplicate timestamp.");
    }
    return;
  }
  const double MAX_ALLOWABLE_GAP = 0.2;
  if (dt > MAX_ALLOWABLE_GAP) {
    RCLCPP_WARN(rclcpp::get_logger("localizer"), "Huge gap detected (%f s), skipping it", dt);
    return;
  }

  //Get raw data
  double raw_vx = velocity.twist.linear.x;
  double raw_vy = velocity.twist.linear.y;
  double raw_vz = velocity.twist.linear.z;
  double omega_z = velocity.twist.angular.z;
  double raw_theta = tf2::getYaw(pose_.orientation);

  //Lag
  const double compass_lag = 0.04;
  double predicted_theta = raw_theta + (omega_z * compass_lag);

  //Calibration
  if (state_ == LocalizerState::CALIBRATING) {
    // Skip the first readings: some from the end of the last game may still be on their way
    if (!has_calibration_start_) {
      calibration_start_ = current_time;
      has_calibration_start_ = true;
    }
    if ((current_time - calibration_start_).seconds() < 0.3) {
      return;
    }

    sum_vx_ += raw_vx;
    sum_vy_ += raw_vy;
    sum_vz_ += raw_vz;
    calibration_count_++;
    return;
  }

  //Subtracting bias'
  double vx = raw_vx - vel_bias_x_;
  double vy = raw_vy - vel_bias_y_;
  double vz = raw_vz - vel_bias_z_;
  double theta = predicted_theta;

  x_ += (vx * std::cos(theta) - vy * std::sin(theta)) * dt;
  y_ += (vx * std::sin(theta) + vy * std::cos(theta)) * dt;
  z_ += vz * dt;

  pose_.position.x = x_;
  pose_.position.y = y_;
  pose_.position.z = z_;

  // remember where we were, so the sonar can place each beam from where it fired
  history_.push_back({current_time.seconds(), x_, y_, z_, theta});
  while (history_.back().t - history_.front().t > kHistoryLength) {
    history_.pop_front();
  }
}

void Localizer::on_imu(const sensor_msgs::msg::Imu & imu)
{
  // The compass gives the heading directly
  pose_.orientation = imu.orientation;
}

void Localizer::on_sonar(const sensor_msgs::msg::LaserScan & scan, const nav_msgs::msg::OccupancyGrid & map)
{
  // TODO(hackathon): bonus! Use the walls the sonar sees to correct the drift
  build_distance_field(map);

  // Put every beam on the map, from where the ghost was and which way it faced when that beam fired
  const double stamp = rclcpp::Time(scan.header.stamp).seconds();
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const double range = scan.ranges[i];
    // inf = no echo, nothing to match
    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
      continue;
    }
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw = 0.0;
    if (!pose_at(stamp + static_cast<double>(i) * scan.time_increment, x, y, z, yaw)) {
      continue;
    }
    // beam angles are from the nose, so add the heading to get the map angle
    const double angle = yaw + scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    endpoints_.emplace_back(x + range * std::cos(angle), y + range * std::sin(angle));
  }

  // a message is a tenth of a turn. Once there's a whole turn, match it against the walls
  turn_angle_ += static_cast<double>(scan.ranges.size()) * scan.angle_increment;
  if (turn_angle_ >= 2.0 * M_PI - 1e-6) {
    match_scan();
    endpoints_.clear();
    turn_angle_ = 0.0;
  }
}

void Localizer::on_depth(const sensor_msgs::msg::Image & depth, const sensor_msgs::msg::CameraInfo & camera_info)
{
  // how high we thought we were when this frame was taken
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
  if (!pose_at(rclcpp::Time(depth.header.stamp).seconds(), x, y, z, yaw)) {
    return;
  }
  const double fy = camera_info.k[4];
  const double cy = camera_info.k[5];
  if (fy <= 0.0) {
    return;
  }

  // Use camera to determine height of rover
  // 10 pixels down a column: on the floor they all say the same height (flat). On a wall they all hit
  // at the same depth, so the height they say keeps changing. Only flat runs count
  const int bin_count = static_cast<int>((kHighestHeight - kLowestHeight) / kHeightBinSize);
  std::vector<int> count(bin_count, 0);
  std::vector<std::pair<double, int>> levels;  // (height, bin) of each flat run
  for (int top = static_cast<int>(depth.height) - kRunRows; top >= 0; top -= kRunRows) {
    // pixel centers are half a pixel in
    if ((top + 0.5 - cy) / fy < kMinDown) {
      break;
    }
    for (int u = 0; u < static_cast<int>(depth.width); ++u) {
      std::array<double, kRunRows> downs{};
      std::array<double, kRunRows> heights{};
      double mean_down = 0.0;
      double mean_height = 0.0;
      bool complete = true;
      for (int i = 0; i < kRunRows; ++i) {
        const float d = depth_at(depth, u, top + i);
        if (!std::isfinite(d) || d <= 0.0f) {
          complete = false;
          break;
        }
        downs[i] = (top + i + 0.5 - cy) / fy;
        heights[i] = d * downs[i] - kCameraUp;
        mean_down += downs[i] / kRunRows;
        mean_height += heights[i] / kRunRows;
      }
      if (!complete) {
        continue;
      }
      // line of best fit through (down, height): flat is a slope of 0, a wall's slope is its depth
      double spread = 0.0;
      double together = 0.0;
      double worst = 0.0;
      for (int i = 0; i < kRunRows; ++i) {
        spread += (downs[i] - mean_down) * (downs[i] - mean_down);
        together += (downs[i] - mean_down) * (heights[i] - mean_height);
        worst = std::max(worst, std::abs(heights[i] - mean_height));
      }
      if (std::abs(together / spread) > kMaxSlope || worst > kMaxFlatError) {
        continue;
      }
      const int bin = static_cast<int>(std::floor((mean_height - kLowestHeight) / kHeightBinSize));
      if (bin < 0 || bin >= bin_count) {
        continue;
      }
      ++count[bin];
      levels.emplace_back(mean_height, bin);
    }
  }

  // The highest height lots of pixels agree on
  int floor_bin = -1;
  for (int bin = bin_count - 2; bin >= 1; --bin) {
    if (count[bin - 1] + count[bin] + count[bin + 1] >= kMinFloorRuns) {
      floor_bin = bin;
      break;
    }
  }
  if (floor_bin < 0) {
    return;  // no floor in view, e.g. right up against a wall
  }
  double sum = 0.0;
  int used = 0;
  for (const auto & [height, bin] : levels) {
    if (std::abs(bin - floor_bin) <= 1) {
      sum += height;
      ++used;
    }
  }
  const double error = sum / used - z;
  // way off: we saw something that isn't the floor, like a table top. Don't trust it
  if (std::abs(error) > kHeightGate) {
    return;
  }

  // nudge, and move the history too, like the sonar does for x and y
  const double dz = kHeightGain * error;
  z_ += dz;
  pose_.position.z = z_;
  for (auto & sample : history_) {
    sample.z += dz;
  }
  RCLCPP_DEBUG(rclcpp::get_logger("localizer"), "Depth: floor in %d runs, nudged z %.3f", used, dz);
}

bool Localizer::pose_at(double t, double & x, double & y, double & z, double & yaw) const
{
  if (history_.size() < 2 || t < history_.front().t) {
    return false;
  }
  // between two samples: blend them. Newer than the last one (a beam can be up to 40 ms ahead of the
  // last velocity): keep going the way the last two were heading, but not for long
  size_t b = history_.size() - 1;
  if (t < history_.back().t) {
    const auto later = std::upper_bound(
      history_.begin(), history_.end(), t, [](double time, const PoseSample & s) { return time < s.t; });
    b = static_cast<size_t>(later - history_.begin());
  } else if (t - history_.back().t > 0.15) {
    return false;
  }
  const PoseSample & s0 = history_[b - 1];
  const PoseSample & s1 = history_[b];
  const double u = (t - s0.t) / (s1.t - s0.t);
  x = s0.x + u * (s1.x - s0.x);
  y = s0.y + u * (s1.y - s0.y);
  z = s0.z + u * (s1.z - s0.z);
  yaw = s0.yaw + u * wrap_angle(s1.yaw - s0.yaw);
  return true;
}

void Localizer::match_scan()
{
  // Find the shift (dx, dy) that puts this turn's points on the walls: Gauss-Newton, a few rounds
  double dx = 0.0;
  double dy = 0.0;
  int inliers = 0;
  for (int round = 0; round < 5; ++round) {
    // the 2x2 system H * step = -g. Damping on the diagonal keeps a corridor from sliding
    double hxx = kDamping;
    double hxy = 0.0;
    double hyy = kDamping;
    double gx = 0.0;
    double gy = 0.0;
    inliers = 0;
    for (const auto & [px, py] : endpoints_) {
      double d = 0.0;
      double ddx = 0.0;
      double ddy = 0.0;
      // too far from any wall: a person, or the wrong wall. Skip it
      if (!distance_field_at(px + dx, py + dy, d, ddx, ddy) || std::abs(d) > kInlierGate) {
        continue;
      }
      // Huber: close points count fully, further ones less
      const double w = std::abs(d) <= kHuberWidth ? 1.0 : kHuberWidth / std::abs(d);
      hxx += w * ddx * ddx;
      hxy += w * ddx * ddy;
      hyy += w * ddy * ddy;
      gx += w * ddx * d;
      gy += w * ddy * d;
      ++inliers;
    }
    if (inliers < kMinInliers) {
      return;
    }
    const double det = hxx * hyy - hxy * hxy;
    const double step_x = -(hyy * gx - hxy * gy) / det;
    const double step_y = -(hxx * gy - hxy * gx) / det;
    dx += step_x;
    dy += step_y;
    // stopped moving, done
    if (std::hypot(step_x, step_y) < 0.001) {
      break;
    }
  }
  // a big jump isn't drift, it's matching the wrong wall
  if (std::hypot(dx, dy) > kInlierGate) {
    return;
  }

  // only go part of the way, and move the history too, so the next turn lines up with this one
  dx *= kCorrectionGain;
  dy *= kCorrectionGain;
  x_ += dx;
  y_ += dy;
  pose_.position.x = x_;
  pose_.position.y = y_;
  for (auto & sample : history_) {
    sample.x += dx;
    sample.y += dy;
  }
  RCLCPP_DEBUG(
    rclcpp::get_logger("localizer"), "Sonar: %d beams on the walls, nudged (%.3f, %.3f)", inliers, dx, dy);
}

void Localizer::build_distance_field(const nav_msgs::msg::OccupancyGrid & map)
{
  // same map as last time -> keep it
  if (map.info == field_info_ && field_.size() == map.data.size()) {
    return;
  }
  field_info_ = map.info;
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  const int reach = static_cast<int>(std::ceil(kFieldReach / map.info.resolution));
  field_.assign(map.data.size(), static_cast<float>(kFieldReach));

  // For every cell, the closest cell of the other kind (free vs blocked). The wall's edge is halfway
  // between their centers, so the distance to the edge is that minus half a cell
  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      const bool blocked = map.data[row * width + col] != 0;
      int best = reach * reach + 1;
      for (int nr = std::max(0, row - reach); nr <= std::min(height - 1, row + reach); ++nr) {
        for (int nc = std::max(0, col - reach); nc <= std::min(width - 1, col + reach); ++nc) {
          const int steps = (nc - col) * (nc - col) + (nr - row) * (nr - row);
          if (steps < best && (map.data[nr * width + nc] != 0) != blocked) {
            best = steps;
          }
        }
      }
      if (best <= reach * reach) {
        const double edge = (std::sqrt(static_cast<double>(best)) - 0.5) * map.info.resolution;
        // + in the open, - inside a wall
        field_[row * width + col] = static_cast<float>(blocked ? -edge : edge);
      } else if (blocked) {
        field_[row * width + col] = static_cast<float>(-kFieldReach);
      }
    }
  }
}

bool Localizer::distance_field_at(double x, double y, double & d, double & gx, double & gy) const
{
  if (field_.empty()) {
    return false;
  }
  const int width = static_cast<int>(field_info_.width);
  const int height = static_cast<int>(field_info_.height);
  const double resolution = field_info_.resolution;
  // values live at cell centers, so blend the 4 centers around the point (bilinear)
  const double fx = (x - field_info_.origin.position.x) / resolution - 0.5;
  const double fy = (y - field_info_.origin.position.y) / resolution - 0.5;
  const int c0 = static_cast<int>(std::floor(fx));
  const int r0 = static_cast<int>(std::floor(fy));
  if (c0 < 0 || r0 < 0 || c0 + 1 >= width || r0 + 1 >= height) {
    return false;
  }
  const double tx = fx - c0;
  const double ty = fy - r0;
  const double v00 = field_[r0 * width + c0];
  const double v10 = field_[r0 * width + c0 + 1];
  const double v01 = field_[(r0 + 1) * width + c0];
  const double v11 = field_[(r0 + 1) * width + c0 + 1];
  d = (1 - tx) * (1 - ty) * v00 + tx * (1 - ty) * v10 + (1 - tx) * ty * v01 + tx * ty * v11;
  // which way it goes up, per meter
  gx = ((v10 - v00) * (1 - ty) + (v11 - v01) * ty) / resolution;
  gy = ((v01 - v00) * (1 - tx) + (v11 - v10) * tx) / resolution;
  return true;
}

}  // namespace ghost_brain
