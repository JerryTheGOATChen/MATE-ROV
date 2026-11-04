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

#include "ghost_brain/controller.hpp"
#include <algorithm>
#include <cmath>

namespace ghost_brain
{

namespace
{

/// Convert orientation quaternion to planar yaw angle in radians
double get_yaw(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

/// Wrap an angle into [-pi, pi]
double compute_angular_error(double current_heading, double target_heading)
{
  double raw_error = target_heading - current_heading;

  return std::remainder(raw_error, 2 * M_PI);
}

/// @return index of the path point closest to (x, y)
size_t closest_index(const Path & path, double x, double y)
{
  // TODO: loop over the path, track the smallest distance
  if (path.empty()) return 0;

  size_t index = 0;
  double min_dist = INFINITY;

  for(size_t i = 0; i < path.size(); i++){
    double current_dist = pow(path[i].x-x, 2) + pow(path[i].y-y,2);
    if(current_dist < min_dist){
      index = i;
      min_dist = current_dist;
    };
  }
  return index;
}

/// @return index of the point about `lookahead` meters down the path from `start`
size_t pick_lookahead(const Path & path, size_t start, double lookahead)
{
  // TODO: walk forward from `start`, adding segment lengths until you pass `lookahead`.
  // If you run out of path, return the last index
  if (path.empty()) return 0;

  double current_dist = 0;

  for(size_t i = start + 1; i < path.size(); ++i){
    current_dist += std::hypot(path[i].x-path[i-1].x, path[i].y-path[i-1].y);
    if(current_dist >= lookahead) return i;
  }

  return path.size()-1;
}

}  // namespace

double Pid::update(double error, double dt)
{
  
  integral += error * dt;
  integral = std::clamp(integral, -1.0, 1.0);

  double derivative = 0.0;

  if(has_prev) derivative = (error - prev_error) / dt;
  prev_error = error;
  has_prev = true;

  return (kp * error) + (ki * integral) + (kd * derivative);
}

void Pid::reset()
{
  has_prev = false;
  prev_error = 0.0;
  integral = 0.0;
}

void Controller::reset()
{
  // TODO: reset each Pid, and clear has_last_goal_
  heading_pid_.reset();
  altitude_pid_.reset();
  has_last_goal_ = false;
}

geometry_msgs::msg::Twist Controller::compute_velocity(const geometry_msgs::msg::Pose & pose, const Path & path)
{
  // TODO(hackathon): fly the ghost along the path
  //  1. Empty path -> reset() and return zeros
  //  2. Get yaw from pose.orientation (tf2::getYaw)
  //  3. Find the closest index, then the lookahead index -> target point
  //  4. Final point within kTolerance -> stop (zeros)
  //  5. Target changed since last tick (compare to last_goal_)? -> reset()
  //  6. dx, dy, dz = target - ghost. Heading error = wrap_angle(atan2(dy, dx) - yaw)
  //  7. angular.z = heading_pid_.update(error, kDt), clamped to +-1.5
  //  8. linear.x from distance, scaled down when the heading error is large, clamped to 1.0
  //  9. linear.z = altitude_pid_.update(dz, kDt), clamped to +-0.6

  geometry_msgs::msg::Twist cmd;

  if(path.empty()){
    reset();
    return cmd;
  } 

  double ghost_x = pose.position.x;
  double ghost_y = pose.position.y;
  double ghost_z = pose.position.z;

  // stop if within kTolerance of end
  const auto & final_point = path.back();
  const double squared_dist_to_end = 
    pow(final_point.x - ghost_x, 2) +
    pow(final_point.y - ghost_y, 2) +
    pow(final_point.z - ghost_z, 2);
  if(squared_dist_to_end < pow(kTolerance, 2)){
    reset();
    return cmd;
  }

  // reset PID if changed goal
  if(!has_last_goal_ || 
     pow(final_point.x - last_goal_.x, 2) + pow(final_point.y - last_goal_.y, 2) > pow(0.1, 2)){
    reset();
    last_goal_ = final_point;
    has_last_goal_ = true;
  }

  size_t start_index = closest_index(path, ghost_x, ghost_y);
  size_t target_index = pick_lookahead(path, start_index, kLookahead);
  const auto & target = path[target_index];

  const double dx = target.x - ghost_x;
  const double dy = target.y - ghost_y;
  const double dyaw = compute_angular_error(get_yaw(pose.orientation), std::atan2(dy, dx));
  const double dz = target.z - ghost_z;

  const double alignment = std::max(0.0, std::cos(dyaw));
  
  cmd.linear.x = std::clamp(1.0 * alignment, -1.0, 1.0);
  cmd.angular.z = std::clamp(heading_pid_.update(dyaw, kDt), -1.5, 1.5);
  cmd.linear.z = std::clamp(altitude_pid_.update(dz, kDt), -0.6, 0.6);

  return cmd;
}

}  // namespace ghost_brain
