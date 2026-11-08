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

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rov_nav2_plugins/costmap/costmap_math.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace rov_nav2_plugins
{

/**
 * @brief Nav2 costmap layer that keeps a 3D voxel grid of the sonar's returns, the floor included, and
 * inflates it in 3D.
 *
 * Nav2's voxel layers, and STVL, flatten every voxel into the 2D costmap, so the floor would make every
 * cell lethal and has to be filtered out. This layer keeps its own 3D grid over the costmap's x, y window
 * and from `min_z` to `max_z`, marks every return, and inflates the grid in 3D (see inflate()). The 3D
 * costs, and which voxels the sonar has seen, are available from cost_volume() to a 3D planner in the
 * same server. Nothing is written into the 2D costmap: the layer uses the costmap only for its window,
 * frame, footprint and update loop.
 *
 * Parameters (under the plugin name):
 *  - `enabled` (bool, default true)
 *  - `topic` (string, default "/sonar/points"): the sensor_msgs/PointCloud2 of returns
 *  - `min_z`, `max_z` (m, default -4.4 and 0.0): the grid's vertical extent in the costmap's frame
 *  - `obstacle_range` (m, default 8.0): returns farther from the sensor are ignored
 *  - `voxel_decay` (s, default 60.0): a voxel not seen for this long is forgotten, becoming unknown
 *  - `robot_half_height` (m, default 0.2): the vehicle's vertical half-height
 *  - `inflation_radius` (m, default 1.0) and `cost_scaling_factor` (1/m, default 3.0), as in Nav2's
 *    InflationLayer. The inscribed radius is the costmap's, from its footprint or `robot_radius`.
 *  - `transform_tolerance` (s, default 0.5): how long to wait for a cloud's transform before dropping it
 *  - `publish_voxel_costs` (bool, default false): publish the inflated voxels on `<plugin name>/voxel_costs`
 */
class VoxelLayer3D : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void reset() override;
  void matchSize() override;

  bool isClearable() override;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x, double * max_y)
    override;

  void updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j) override;

  /// The latest 3D costs, or null before the costmap has a size. The volume never changes once returned.
  std::shared_ptr<const CostVolume> cost_volume() const;

private:
  void on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);

  /// Transform `cloud` into the costmap's frame and add it to the grid. False if its transform is not available yet.
  bool insert_cloud(const sensor_msgs::msg::PointCloud2 & cloud);

  void publish_voxel_costs(const CostVolume & volume);

  double min_z_{-4.4};
  double max_z_{0.0};
  double obstacle_range_{8.0};
  double voxel_decay_{60.0};
  double robot_half_height_{0.2};
  double inflation_radius_{1.0};
  double cost_scaling_factor_{3.0};
  double transform_tolerance_{0.5};

  std::optional<VoxelGrid3D> grid_;
  bool grid_changed_{false};  // since the cost volume was last computed

  std::mutex clouds_mutex_;
  std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> clouds_;  // received, not yet inserted

  mutable std::mutex volume_mutex_;
  std::shared_ptr<const CostVolume> volume_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_costs_pub_;
};

}  // namespace rov_nav2_plugins
