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
#include "pluginlib/class_loader.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.hpp"

namespace rov_nav2_plugins
{

/**
 * @brief Nav2 global planner that routes around the costmap's obstacles in 3D.
 *
 * A stock Nav2 2D planner finds the horizontal route through the costmap, and add_depth() adds depth
 * and yaw to it.
 *
 * Parameters (under the plugin name):
 *  - `xy_planner` (string, default "nav2_smac_planner::SmacPlanner2D"): the 2D planner plugin. Its own
 *    parameters go under `<plugin name>.xy`, such as `CostmapPlanner3D.xy.tolerance`.
 *  - `z_before_xy` (bool, default false): change depth first, then move horizontally
 */
class CostmapPlanner3D : public nav2_core::GlobalPlanner
{
public:
  CostmapPlanner3D();
  ~CostmapPlanner3D() override;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  /**
   * @throws nav2_core::PlannerException subclasses from the 2D planner, such as
   *   nav2_core::NoValidPathCouldBeFound when obstacles block every route
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

private:
  bool z_before_xy_{false};

  // The loader must outlive the planner it created, so it is declared first
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader_;
  pluginlib::UniquePtr<nav2_core::GlobalPlanner> xy_planner_;
};

}  // namespace rov_nav2_plugins
