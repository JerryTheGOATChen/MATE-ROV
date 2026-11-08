// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
#include "rov_nav2_plugins/planner/octomap_planner_3d.hpp"

#include <algorithm>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>

#include <angles/angles.h>
#include <octomap/AbstractOcTree.h>
#include <octomap_msgs/conversions.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "nav2_core/planner_exceptions.hpp"
#include "nav2_util/node_utils.hpp"

namespace rov_nav2_plugins
{

void OctomapPlanner3D::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  (void)tf;
  (void)costmap_ros;
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("OctomapPlanner3D: unable to lock the parent node");
  }
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  const std::string octomap_topic = nav2_util::declare_or_get_parameter(
    node, name + ".octomap_topic", std::string("/octomap_binary"));
  map_update_interval_ = nav2_util::declare_or_get_parameter(
    node, name + ".map_update_interval", 0.5);
  if (map_update_interval_ < 0.0) {
    throw std::runtime_error("OctomapPlanner3D.map_update_interval cannot be negative");
  }
  const std::vector<double> min_bounds = nav2_util::declare_or_get_parameter(
    node, name + ".min_bounds", std::vector<double>{-11.0, -7.0, -4.0});
  const std::vector<double> max_bounds = nav2_util::declare_or_get_parameter(
    node, name + ".max_bounds", std::vector<double>{11.0, 7.0, 0.0});
  if (min_bounds.size() != 3U || max_bounds.size() != 3U) {
    throw std::runtime_error("OctomapPlanner3D bounds must each contain [x, y, z]");
  }

  OctomapPlannerConfig config;
  config.planning_resolution = nav2_util::declare_or_get_parameter(
    node, name + ".planning_resolution", 0.25);
  config.collision_radius = nav2_util::declare_or_get_parameter(
    node, name + ".collision_radius", 0.45);
  config.min_bounds = Eigen::Vector3d(min_bounds[0], min_bounds[1], min_bounds[2]);
  config.max_bounds = Eigen::Vector3d(max_bounds[0], max_bounds[1], max_bounds[2]);
  config.unknown_is_occupied = nav2_util::declare_or_get_parameter(
    node, name + ".unknown_is_occupied", false);
  const int max_expansions = nav2_util::declare_or_get_parameter(
    node, name + ".max_expansions", 250000);
  if (max_expansions <= 0) {
    throw std::runtime_error("OctomapPlanner3D.max_expansions must be positive");
  }
  config.max_expansions = static_cast<std::size_t>(max_expansions);
  planner_ = std::make_unique<OctomapPlanner>(config);

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  octomap_subscription_ = node->create_subscription<octomap_msgs::msg::Octomap>(
    octomap_topic,
    qos,
    std::bind(&OctomapPlanner3D::octomapCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    logger_,
    "Configured %s for true 3D OctoMap planning on %s "
    "(resolution %.2f m, radius %.2f m, map interval %.2f s)",
    name.c_str(), octomap_topic.c_str(), config.planning_resolution,
    config.collision_radius, map_update_interval_);
}

void OctomapPlanner3D::cleanup()
{
  octomap_subscription_.reset();
  planner_.reset();
  std::lock_guard<std::mutex> lock(octomap_mutex_);
  octree_.reset();
  octomap_frame_.clear();
  last_octomap_stamp_ns_ = 0;
}

void OctomapPlanner3D::activate() {}

void OctomapPlanner3D::deactivate() {}

void OctomapPlanner3D::octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
  const std::int64_t stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
  {
    std::lock_guard<std::mutex> lock(octomap_mutex_);
    const std::int64_t minimum_interval_ns = static_cast<std::int64_t>(
      map_update_interval_ * 1'000'000'000.0);
    if (minimum_interval_ns > 0 && stamp_ns > 0 && last_octomap_stamp_ns_ > 0 &&
      stamp_ns >= last_octomap_stamp_ns_ &&
      stamp_ns - last_octomap_stamp_ns_ < minimum_interval_ns)
    {
      return;
    }
  }

  std::unique_ptr<octomap::AbstractOcTree> abstract_tree(octomap_msgs::msgToMap(*msg));
  if (!abstract_tree) {
    RCLCPP_WARN(logger_, "Could not deserialize an OctoMap update");
    return;
  }
  auto * cast_tree = dynamic_cast<octomap::OcTree *>(abstract_tree.get());
  if (cast_tree == nullptr) {
    RCLCPP_WARN(logger_, "OctoMap type '%s' is not an OcTree", msg->id.c_str());
    return;
  }

  std::shared_ptr<octomap::OcTree> new_tree(
    static_cast<octomap::OcTree *>(abstract_tree.release()));
  std::lock_guard<std::mutex> lock(octomap_mutex_);
  octree_ = std::move(new_tree);
  octomap_frame_ = msg->header.frame_id;
  last_octomap_stamp_ns_ = stamp_ns;
}

nav_msgs::msg::Path OctomapPlanner3D::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  std::shared_ptr<octomap::OcTree> tree;
  std::string map_frame;
  {
    std::lock_guard<std::mutex> lock(octomap_mutex_);
    tree = octree_;
    map_frame = octomap_frame_;
  }
  if (!tree) {
    throw nav2_core::NoValidPathCouldBeFound(
            "No OctoMap has arrived yet; check /sonar/points and /octomap_binary");
  }
  if (!map_frame.empty() && map_frame != start.header.frame_id) {
    throw nav2_core::NoValidPathCouldBeFound(
            "OctoMap frame '" + map_frame + "' does not match planner frame '" +
            start.header.frame_id + "'");
  }

  const Eigen::Vector3d start_point(
    start.pose.position.x, start.pose.position.y, start.pose.position.z);
  const Eigen::Vector3d goal_point(
    goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
  const auto points = planner_->plan(*tree, start_point, goal_point, cancel_checker);
  if (points.empty()) {
    throw nav2_core::NoValidPathCouldBeFound(
            "No collision-free 3D path was found through the OctoMap");
  }

  std::vector<double> distance_along(points.size(), 0.0);
  for (std::size_t i = 1; i < points.size(); ++i) {
    distance_along[i] = distance_along[i - 1] + (points[i] - points[i - 1]).norm();
  }
  const double start_yaw = tf2::getYaw(start.pose.orientation);
  const double yaw_change = angles::shortest_angular_distance(
    start_yaw, tf2::getYaw(goal.pose.orientation));
  const double total_distance = distance_along.back();

  nav_msgs::msg::Path path;
  path.header.frame_id = start.header.frame_id;
  path.header.stamp = clock_->now();
  path.poses.reserve(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = points[i].x();
    pose.pose.position.y = points[i].y();
    pose.pose.position.z = points[i].z();
    const double fraction = total_distance > 0.0 ? distance_along[i] / total_distance : 1.0;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, start_yaw + fraction * yaw_change);
    pose.pose.orientation = tf2::toMsg(orientation);
    path.poses.push_back(std::move(pose));
  }
  return path;
}

}  // namespace rov_nav2_plugins

PLUGINLIB_EXPORT_CLASS(rov_nav2_plugins::OctomapPlanner3D, nav2_core::GlobalPlanner)
