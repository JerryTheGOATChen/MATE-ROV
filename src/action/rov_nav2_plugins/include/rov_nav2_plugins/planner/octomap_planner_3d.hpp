// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
#pragma once

#include <cstdint>
#include <limits>
#include <memory>
#include <shared_mutex>
#include <string>

#include <message_filters/subscriber.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/message_filter.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "nav2_core/global_planner.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rov_nav2_plugins/planner/octomap_mapper.hpp"
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
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud);
  void publishOccupiedCells(const builtin_interfaces::msg::Time & stamp);

  rclcpp::Logger logger_{rclcpp::get_logger("OctomapPlanner3D")};
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  message_filters::Subscriber<
    sensor_msgs::msg::PointCloud2,
    rclcpp_lifecycle::LifecycleNode> pointcloud_subscription_;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> pointcloud_filter_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    occupied_cells_publisher_;
  std::unique_ptr<OctomapPlanner> planner_;
  std::unique_ptr<OctomapMapper> mapper_;

  mutable std::shared_mutex octomap_mutex_;
  std::uint64_t map_revision_{0};
  std::string map_frame_{"map"};
  double pointcloud_min_x_{-std::numeric_limits<double>::max()};
  double pointcloud_max_x_{std::numeric_limits<double>::max()};
  double pointcloud_min_y_{-std::numeric_limits<double>::max()};
  double pointcloud_max_y_{std::numeric_limits<double>::max()};
  double pointcloud_min_z_{-100.0};
  double pointcloud_max_z_{100.0};
  bool visualization_enabled_{true};
  double visualization_interval_{1.0};
  double visualization_min_z_{-4.0};
  double visualization_max_z_{0.0};
  std::int64_t last_visualization_stamp_ns_{0};
};

}  // namespace rov_nav2_plugins
