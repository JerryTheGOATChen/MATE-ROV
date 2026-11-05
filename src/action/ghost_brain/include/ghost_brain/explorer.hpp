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

#pragma once

#include <optional>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace ghost_brain
{
// How long looking somewhere lasts
constexpr double kFadeTime = 25.0;

/**
 * @brief Remembers where the camera has looked lately, so the ghost can go look somewhere else.
 *
 * It keeps one "last seen" time for every cell of /map: same cells, same order (row by row), so
 * the row/col math from the planner works here too, and so does the lesson 8 flood.
 */
/// Explorer numbers you can tune in config/ghost_brain.yaml
struct ExplorerSettings
{
    // seconds until somewhere it looked counts as unseen again. Lower = patrols more
    double fade_time{kFadeTime};
};

class Explorer
{
public:
    void set_settings(const ExplorerSettings & settings) { settings_ = settings; }
    const ExplorerSettings & settings() const { return settings_; }

    // Forget everything. Called at the start of every game
    void reset();

    // Mark the cells this depth frame shows. Called for every depth frame
    void on_depth(
        const sensor_msgs::msg::Image & depth,
        const sensor_msgs::msg::CameraInfo & camera_info,
        const geometry_msgs::msg::Pose & ghost_pose,
        const nav_msgs::msg::OccupancyGrid & map);

    // Whether the camera looked at this point in the last kFadeTime seconds
    bool is_seen(double x, double y, const rclcpp::Time & now) const;

    // Closest place it hasn't looked at lately
    std::optional<geometry_msgs::msg::Point> next_goal(
        const std::vector<double> & distances,
        const geometry_msgs::msg::Pose & ghost_pose,
        const rclcpp::Time & now) const;

    // What it has seen as a grid for Foxglove: 100 = seen lately, 0 = not
    nav_msgs::msg::OccupancyGrid seen_grid(const rclcpp::Time & now) const;

    private:
    // make last_seen_ the same size as the map, if it isn't yet
    void fit_to(const nav_msgs::msg::OccupancyGrid & map);
    // the cell a point is in, or -1 if it's off the map
    int cell_at(double x, double y) const;

    // the map's size and where it starts
    nav_msgs::msg::MapMetaData info_;
    // when each cell was last seen, in seconds. -infinity = never
    std::vector<double> last_seen_;
    ExplorerSettings settings_;
};
}
