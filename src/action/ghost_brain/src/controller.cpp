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
#include <cstddef>
#include <limits>

#include "tf2/utils.hpp"

namespace ghost_brain
{

geometry_msgs::msg::Twist Controller::compute_velocity(const geometry_msgs::msg::Pose & pose, const Path & path)
{
  //Pure pursuit controller

  //Empty path
  if (path.empty()) {
    return geometry_msgs::msg::Twist{};
  }

  double chase_x = path.back().x;     //Default to end goal
  double chase_y = path.back().y;
  double chase_z = path.back().z;

  if (path.size() > 1) {
    double min_dist_sq = std::numeric_limits<double>::max();
    size_t best_i = 0;
    double closest_x = 0.0;
    double closest_y = 0.0;
    double closest_z = 0.0;

    //Finding closest point (lin alg)
    for (size_t i = 0; i < path.size() - 1; ++i) {
      const auto & pose_a = path[i];
      const auto & pose_b = path[i + 1];
      const auto & pose_p = pose.position;

      //Finding path vector ab
      const double vx = pose_b.x - pose_a.x;
      const double vy = pose_b.y - pose_a.y;

      //Relative robot vector ap, p is point of robot
      const double wx = pose_p.x - pose_a.x;
      const double wy = pose_p.y - pose_a.y;

      //Project
      const double dot_product = wx * vx + wy * vy;
      const double v_mag_sqrd = vx * vx + vy * vy;

      //Avoid dividing by 0
      double t = 0.0;
      if (v_mag_sqrd > 0.0) {
        //Clamping from 0.0 - 1.0
        t = std::clamp(dot_product / v_mag_sqrd, 0.0, 1.0);
      }
      //Finding closest point along path
      const double cx = pose_a.x + t * vx;
      const double cy = pose_a.y + t * vy;
      const double cz = pose_a.z + t * (pose_b.z - pose_a.z);

      //Finding closest point for all little paths
      const double dist_sq = (pose_p.x - cx) * (pose_p.x - cx) + (pose_p.y - cy) * (pose_p.y - cy);
      if (dist_sq < min_dist_sq) {
        min_dist_sq = dist_sq;
        best_i = i;
        closest_x = cx;
        closest_y = cy;
        closest_z = cz;
      }
    }

    //Lookahead dist of 0.6 to 0.8 decide later
    const double lookahead_dist = settings_.lookahead_dist;

    //Using same best segment from closest point (same pose_b as before)
    const auto & pose_b = path[best_i + 1];
    const double seg_dx = pose_b.x - closest_x;
    const double seg_dy = pose_b.y - closest_y;
    const double seg_dz = pose_b.z - closest_z;
    const double seg_dist = std::sqrt(seg_dx * seg_dx + seg_dy * seg_dy);

    double accumulated_dist = seg_dist;

    if (accumulated_dist >= lookahead_dist) {
      // Lookahead point is on this very first segment
      const double ratio = lookahead_dist / seg_dist;
      chase_x = closest_x + ratio * seg_dx;
      chase_y = closest_y + ratio * seg_dy;
      chase_z = closest_z + ratio * seg_dz;
    } else {
      //Lookahead isn't in this segment, look for correct segment
      for (size_t target_i = best_i + 1; target_i < path.size() - 1; ++target_i) {
        const auto & sa = path[target_i];
        const auto & sb = path[target_i + 1];
        const double sdx = sb.x - sa.x;
        const double sdy = sb.y - sa.y;
        const double sdist = std::sqrt(sdx * sdx + sdy * sdy);

        if (accumulated_dist + sdist >= lookahead_dist) {
          // Lookahead point is on this segment!
          const double ratio = (lookahead_dist - accumulated_dist) / sdist;
          chase_x = sa.x + ratio * sdx;
          chase_y = sa.y + ratio * sdy;
          chase_z = sa.z + ratio * (sb.z - sa.z);
          break;
        }
        accumulated_dist += sdist;
      }
    }
  }

  //Calculate Twist command
  const double map_dx = chase_x - pose.position.x;
  const double map_dy = chase_y - pose.position.y;
  const double map_dz = chase_z - pose.position.z;
  const double theta = tf2::getYaw(pose.orientation);

  const double ghost_dx = map_dx * std::cos(theta) + map_dy * std::sin(theta);
  const double ghost_dy = map_dy * std::cos(theta) - map_dx * std::sin(theta);

  //some pid stuff
  const double k_p = 2.0;
  double target_vx = ghost_dx * k_p;
  double target_vy = ghost_dy * k_p;
  double target_vz = map_dz * k_p;

  //clamp
  // Slow every axis by the same amount to fit the limits
  const double scale = std::min(
    {1.0, 1.0 / std::abs(target_vx), 0.6 / std::abs(target_vy), 0.6 / std::abs(target_vz)});
  target_vx *= scale;
  target_vy *= scale;
  target_vz *= scale;

  //Rotation control
  // Right on top of the chase point the direction to it is just noise
  double target_yaw = 0.0;
  if (std::hypot(map_dx, map_dy) > 0.2) {
    const double target_angle = std::atan2(map_dy, map_dx);
    double angle_error = target_angle - theta;
    // Normalize angle error
    while (angle_error > M_PI) {
      angle_error -= 2.0 * M_PI;
    }
    while (angle_error < -M_PI) {
      angle_error += 2.0 * M_PI;
    }
    target_yaw = std::clamp(angle_error * 2.0, -1.5, 1.5);  // P-gain for rotation
  }

  geometry_msgs::msg::Twist twist;
  twist.linear.x = target_vx;
  twist.linear.y = target_vy;
  twist.linear.z = target_vz;
  twist.angular.z = target_yaw;
  return twist;
}

}  // namespace ghost_brain
