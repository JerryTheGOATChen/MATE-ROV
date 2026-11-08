// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <octomap/OcTree.h>
#include <octomap_msgs/msg/octomap.hpp>

#include "nav2_core/global_planner.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rov_nav2_plugins/planner/octomap_planner.hpp"

namespace rov_nav2_plugins
{

/** @brief Nav2 global planner that searches an OctoMap in x, y and z. */
class OctomapPlanner3D : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

private:
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);

  rclcpp::Logger logger_{rclcpp::get_logger("OctomapPlanner3D")};
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_subscription_;
  std::unique_ptr<OctomapPlanner> planner_;

  std::mutex octomap_mutex_;
  std::shared_ptr<octomap::OcTree> octree_;
  std::string octomap_frame_;
  double map_update_interval_{0.5};
  std::int64_t last_octomap_stamp_ns_{0};
};

}  // namespace rov_nav2_plugins
