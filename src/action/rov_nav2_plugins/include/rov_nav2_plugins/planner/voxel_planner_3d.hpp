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

#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rov_nav2_plugins/costmap/voxel_layer_3d.hpp"
#include "rov_nav2_plugins/planner/planner_math.hpp"
#include "tf2_ros/buffer.hpp"

namespace rov_nav2_plugins
{

/**
 * @brief Nav2 global planner that routes over, under or around obstacles through VoxelLayer3D's 3D costs.
 *
 * The layer must be in the planner server's costmap, the global costmap. Each plan reads the layer's latest
 * cost volume, plans through it with plan_through_voxels(), and adds headings with add_yaw().
 *
 * Parameters (under the plugin name):
 *  - `voxel_layer` (string, default "voxel_layer_3d"): the name of the VoxelLayer3D in the costmap
 *  - `cost_weight` (default 2.0), `unknown_cost` (int, default 40), `z_weight` (default 1.5),
 *    `max_planning_time` (s, default 1.0): see PlannerConfig
 *  - `yaw_mode` (string, default "interpolate"): "interpolate" or "face_travel", see YawMode
 */
class VoxelPlanner3D : public nav2_core::GlobalPlanner
{
public:
  /// @throws std::runtime_error if the costmap has no VoxelLayer3D named `voxel_layer`
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  /**
   * @throws nav2_core::PlannerException subclasses: StartOutsideMapBounds, GoalOutsideMapBounds,
   *   StartOccupied, GoalOccupied, NoValidPathCouldBeFound, PlannerTimedOut and PlannerCancelled, or the base
   *   class if the layer's grid has no size
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

private:
  rclcpp::Logger logger_{rclcpp::get_logger("VoxelPlanner3D")};
  rclcpp::Clock::SharedPtr clock_;
  std::string global_frame_;
  std::shared_ptr<VoxelLayer3D> layer_;
  PlannerConfig config_;
  YawMode yaw_mode_{YawMode::kInterpolate};
};

}  // namespace rov_nav2_plugins
