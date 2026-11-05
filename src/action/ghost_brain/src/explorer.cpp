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

#include "ghost_brain/explorer.hpp"

#include <cmath>
#include <limits>

#include "ghost_brain/tracker.hpp"  // depth_at and kCamera*
#include "tf2/utils.hpp"            // tf2::getYaw

namespace ghost_brain
{

void Explorer::reset()
{
    last_seen_.assign(last_seen_.size(), -std::numeric_limits<double>::infinity());
}

void Explorer::fit_to(const nav_msgs::msg::OccupancyGrid & map)
{
    if (map.info == info_ && last_seen_.size() == map.data.size()) {
        return;
    }
    info_ = map.info;
    last_seen_.assign(map.data.size(), -std::numeric_limits<double>::infinity());
}

int Explorer::cell_at(double x, double y) const
{
    if (last_seen_.empty()) {
        return -1;
    }
    const int col = static_cast<int>(std::floor((x - info_.origin.position.x) / info_.resolution));
    const int row = static_cast<int>(std::floor((y - info_.origin.position.y) / info_.resolution));
    if (col < 0 || row < 0 || col >= static_cast<int>(info_.width) || row >= static_cast<int>(info_.height)) {
        return -1;
    }
    return row * static_cast<int>(info_.width) + col;
}

void Explorer::on_depth(
    const sensor_msgs::msg::Image & depth,
    const sensor_msgs::msg::CameraInfo & camera_info,
    const geometry_msgs::msg::Pose & ghost_pose,
    const nav_msgs::msg::OccupancyGrid & map)
{
    fit_to(map);
    // when this frame was taken. Seconds as a double, so no clock type mixups
    const double stamp = rclcpp::Time(depth.header.stamp).seconds();
    const double ghost_x = ghost_pose.position.x;
    const double ghost_y = ghost_pose.position.y;
    const double res = map.info.resolution;

    //Marks everything around 0.5 rad of ghost cause that's absorbtion range
    for (double dx = -0.5; dx <= 0.5; dx += res * 0.5) {
        for (double dy = -0.5; dy <= 0.5; dy += res * 0.5) {
            if (dx * dx + dy * dy <= 0.25) {
                int idx = cell_at(ghost_x + dx, ghost_y + dy);
                if (idx >= 0 && idx < static_cast<int>(last_seen_.size())) {
                    last_seen_[idx] = stamp;
                }
            }
        }
    }

    //Camera stuff same as before
    const double f = camera_info.k[0];
    const double cx = camera_info.k[2];
    const int v = depth.height / 2;

    const double yaw = tf2::getYaw(ghost_pose.orientation);
    const double cam_x = ghost_x + kCameraForward * std::cos(yaw);
    const double cam_y = ghost_y + kCameraForward * std::sin(yaw);

    //Look at every other column
    for (int u = 0; u < static_cast<int>(depth.width); u += 2) {
        double d = depth_at(depth, u, v);
        if (!std::isfinite(d)) { //Nothing blocking image at least 8m
            d = 8.0;
        }

        const double u_term = (static_cast<double>(u) - cx) / f;
        const double angle_offset = std::atan(u_term);
        const double ray_angle = yaw - angle_offset;
        
        const double max_dist = std::min(d * std::sqrt(1.0 + u_term * u_term), 8.0);

        //Steps at 0.5 to ensure every cell gets marked
        const double step = res * 0.5;
        for (double r = 0.5; r <= max_dist; r += step) {
            double wx = cam_x + r * std::cos(ray_angle);
            double wy = cam_y + r * std::sin(ray_angle);
            int idx = cell_at(wx, wy);
            if (idx >= 0 && idx < static_cast<int>(last_seen_.size())) {
                last_seen_[idx] = stamp;
            }
        }
    }
}

bool Explorer::is_seen(double x, double y, const rclcpp::Time & now) const
{
    const int cell = cell_at(x, y);
    if (cell < 0) {
        return false;
    }
    const double elapsed = now.seconds() - last_seen_[cell];
    return elapsed < settings_.fade_time;
}

std::optional<geometry_msgs::msg::Point> Explorer::next_goal(
    const std::vector<double> & distances,
    const geometry_msgs::msg::Pose & ghost_pose,
    const rclcpp::Time & now) const
{
    // no depth frame yet, or the flood is for some other map
    if (last_seen_.empty() || distances.size() != last_seen_.size()) {
        return std::nullopt;
    }
    double min_dist = std::numeric_limits<double>::infinity();
    int best_cell = -1;
    
    //Loop over every cell
    for (size_t i = 0; i < distances.size(); ++i) {
        // Skip if distance is infinite/non-finite (can't get there)
        if (!std::isfinite(distances[i])) {
            continue;
        }
        // Skip if it was seen lately (fade time check)
        if ((now.seconds() - last_seen_[i]) < settings_.fade_time) {
            continue;
        }
        // Keep the one with the smallest distance
        if (distances[i] < min_dist) {
            min_dist = distances[i];
            best_cell = i;
        }
    }

    //Nothing found
    if (best_cell < 0) {
        return std::nullopt;
    }

    //Return point at middle of the cell
    const int col = best_cell % static_cast<int>(info_.width);
    const int row = best_cell / static_cast<int>(info_.width);

    geometry_msgs::msg::Point goal;
    goal.x = info_.origin.position.x + (static_cast<double>(col) + 0.5) * info_.resolution;
    goal.y = info_.origin.position.y + (static_cast<double>(row) + 0.5) * info_.resolution;
    goal.z = 1.4; // keep z cruising at 1.4m

    return goal;
}

nav_msgs::msg::OccupancyGrid Explorer::seen_grid(const rclcpp::Time & now) const
{
    nav_msgs::msg::OccupancyGrid grid;
    grid.info = info_;
    grid.data.assign(last_seen_.size(), 0);
    // asks is_seen about the middle of every cell, so it shows whatever your is_seen says
    for (size_t i = 0; i < last_seen_.size(); ++i) {
        const double x = info_.origin.position.x + (static_cast<double>(i % info_.width) + 0.5) * info_.resolution;
        const double y = info_.origin.position.y + (static_cast<double>(i / info_.width) + 0.5) * info_.resolution;
        grid.data[i] = is_seen(x, y, now) ? 100 : 0;
    }
    return grid;
}

}  // namespace ghost_brain
