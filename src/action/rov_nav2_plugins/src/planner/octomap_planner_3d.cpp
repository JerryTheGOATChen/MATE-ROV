// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
#include "rov_nav2_plugins/planner/octomap_planner_3d.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <vector>

#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

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
  (void)costmap_ros;
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("OctomapPlanner3D: unable to lock the parent node");
  }
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  tf_buffer_ = std::move(tf);
  if (!tf_buffer_) {
    throw std::runtime_error("OctomapPlanner3D: planner server did not provide a TF buffer");
  }

  const std::string pointcloud_topic = nav2_util::declare_or_get_parameter(
    node, name + ".pointcloud_topic", std::string("/sonar/points"));
  map_frame_ = nav2_util::declare_or_get_parameter(
    node, name + ".map_frame", std::string("map"));
  const int tf_queue_size = nav2_util::declare_or_get_parameter(
    node, name + ".tf_queue_size", 5);
  const double tf_filter_tolerance = nav2_util::declare_or_get_parameter(
    node, name + ".tf_filter_tolerance", 5.0);
  if (tf_queue_size <= 0 || tf_filter_tolerance < 0.0) {
    throw std::runtime_error("OctomapPlanner3D TF queue size/tolerance is invalid");
  }

  OctomapMapperConfig mapper_config;
  mapper_config.resolution = nav2_util::declare_or_get_parameter(
    node, name + ".map_resolution", 0.1);
  mapper_config.max_range = nav2_util::declare_or_get_parameter(
    node, name + ".sensor_model.max_range", 8.0);
  mapper_config.probability_hit = nav2_util::declare_or_get_parameter(
    node, name + ".sensor_model.hit", 0.7);
  mapper_config.probability_miss = nav2_util::declare_or_get_parameter(
    node, name + ".sensor_model.miss", 0.4);
  mapper_config.clamping_min = nav2_util::declare_or_get_parameter(
    node, name + ".sensor_model.min", 0.12);
  mapper_config.clamping_max = nav2_util::declare_or_get_parameter(
    node, name + ".sensor_model.max", 0.97);
  mapper_config.occupancy_threshold = nav2_util::declare_or_get_parameter(
    node, name + ".sensor_model.occupancy", 0.5);
  mapper_config.compress_map = nav2_util::declare_or_get_parameter(
    node, name + ".compress_map", true);
  mapper_ = std::make_unique<OctomapMapper>(mapper_config);

  const double unbounded_min = -std::numeric_limits<double>::max();
  const double unbounded_max = std::numeric_limits<double>::max();
  pointcloud_min_x_ = nav2_util::declare_or_get_parameter(
    node, name + ".pointcloud_min_x", unbounded_min);
  pointcloud_max_x_ = nav2_util::declare_or_get_parameter(
    node, name + ".pointcloud_max_x", unbounded_max);
  pointcloud_min_y_ = nav2_util::declare_or_get_parameter(
    node, name + ".pointcloud_min_y", unbounded_min);
  pointcloud_max_y_ = nav2_util::declare_or_get_parameter(
    node, name + ".pointcloud_max_y", unbounded_max);
  pointcloud_min_z_ = nav2_util::declare_or_get_parameter(
    node, name + ".pointcloud_min_z", -100.0);
  pointcloud_max_z_ = nav2_util::declare_or_get_parameter(
    node, name + ".pointcloud_max_z", 100.0);
  if (pointcloud_min_x_ > pointcloud_max_x_ || pointcloud_min_y_ > pointcloud_max_y_ ||
    pointcloud_min_z_ > pointcloud_max_z_)
  {
    throw std::runtime_error("OctomapPlanner3D point-cloud filter bounds are invalid");
  }

  const std::vector<double> min_bounds = nav2_util::declare_or_get_parameter(
    node, name + ".min_bounds", std::vector<double>{-11.0, -7.0, -4.0});
  const std::vector<double> max_bounds = nav2_util::declare_or_get_parameter(
    node, name + ".max_bounds", std::vector<double>{11.0, 7.0, 0.0});
  if (min_bounds.size() != 3U || max_bounds.size() != 3U) {
    throw std::runtime_error("OctomapPlanner3D bounds must each contain [x, y, z]");
  }

  OctomapPlannerConfig planner_config;
  planner_config.planning_resolution = nav2_util::declare_or_get_parameter(
    node, name + ".planning_resolution", 0.25);
  planner_config.collision_radius = nav2_util::declare_or_get_parameter(
    node, name + ".collision_radius", 0.45);
  planner_config.min_bounds = Eigen::Vector3d(min_bounds[0], min_bounds[1], min_bounds[2]);
  planner_config.max_bounds = Eigen::Vector3d(max_bounds[0], max_bounds[1], max_bounds[2]);
  planner_config.unknown_is_occupied = nav2_util::declare_or_get_parameter(
    node, name + ".unknown_is_occupied", false);
  const int max_expansions = nav2_util::declare_or_get_parameter(
    node, name + ".max_expansions", 250000);
  if (max_expansions <= 0) {
    throw std::runtime_error("OctomapPlanner3D.max_expansions must be positive");
  }
  planner_config.max_expansions = static_cast<std::size_t>(max_expansions);
  planner_ = std::make_unique<OctomapPlanner>(planner_config);

  visualization_enabled_ = nav2_util::declare_or_get_parameter(
    node, name + ".visualization.enabled", true);
  visualization_interval_ = nav2_util::declare_or_get_parameter(
    node, name + ".visualization.publish_interval", 1.0);
  visualization_min_z_ = nav2_util::declare_or_get_parameter(
    node, name + ".visualization.min_z", -4.0);
  visualization_max_z_ = nav2_util::declare_or_get_parameter(
    node, name + ".visualization.max_z", 0.0);
  const std::string visualization_topic = nav2_util::declare_or_get_parameter(
    node, name + ".visualization.topic", std::string("/occupied_cells_vis_array"));
  if (visualization_interval_ < 0.0 || visualization_min_z_ > visualization_max_z_) {
    throw std::runtime_error("OctomapPlanner3D visualization parameters are invalid");
  }
  if (visualization_enabled_) {
    occupied_cells_publisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      visualization_topic, rclcpp::QoS(1).reliable().transient_local());
  }

  pointcloud_subscription_.subscribe(
    node.get(), pointcloud_topic, rclcpp::SensorDataQoS().get_rmw_qos_profile());
  pointcloud_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
    pointcloud_subscription_, *tf_buffer_, map_frame_, static_cast<std::uint32_t>(tf_queue_size),
    node->get_node_logging_interface(), node->get_node_clock_interface(),
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(tf_filter_tolerance)));
  pointcloud_filter_->registerCallback(
    std::bind(&OctomapPlanner3D::pointCloudCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    logger_,
    "Configured %s to map %s directly into one persistent %.2f m OctoMap in %s "
    "(max range %.2f m, planning resolution %.2f m, radius %.2f m)",
    name.c_str(), pointcloud_topic.c_str(), mapper_config.resolution, map_frame_.c_str(),
    mapper_config.max_range, planner_config.planning_resolution, planner_config.collision_radius);
}

void OctomapPlanner3D::cleanup()
{
  pointcloud_filter_.reset();
  pointcloud_subscription_.unsubscribe();
  occupied_cells_publisher_.reset();
  planner_.reset();
  tf_buffer_.reset();
  std::unique_lock<std::shared_mutex> lock(octomap_mutex_);
  mapper_.reset();
  map_revision_ = 0;
  last_visualization_stamp_ns_ = 0;
}

void OctomapPlanner3D::activate()
{
  if (occupied_cells_publisher_) {
    occupied_cells_publisher_->on_activate();
  }
}

void OctomapPlanner3D::deactivate()
{
  if (occupied_cells_publisher_) {
    occupied_cells_publisher_->on_deactivate();
  }
}

void OctomapPlanner3D::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
{
  geometry_msgs::msg::TransformStamped sensor_to_map_msg;
  try {
    sensor_to_map_msg = tf_buffer_->lookupTransform(
      map_frame_, cloud->header.frame_id, cloud->header.stamp,
      rclcpp::Duration::from_seconds(1.0));
  } catch (const tf2::TransformException & error) {
    RCLCPP_WARN(
      logger_, "Could not transform sonar cloud into %s: %s", map_frame_.c_str(), error.what());
    return;
  }

  tf2::Transform sensor_to_map;
  tf2::fromMsg(sensor_to_map_msg.transform, sensor_to_map);
  const tf2::Vector3 origin_tf = sensor_to_map.getOrigin();
  const octomap::point3d sensor_origin(
    static_cast<float>(origin_tf.x()),
    static_cast<float>(origin_tf.y()),
    static_cast<float>(origin_tf.z()));

  octomap::Pointcloud endpoints;
  endpoints.reserve(static_cast<std::size_t>(cloud->width) * cloud->height);
  try {
    sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(*cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(*cloud, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
        continue;
      }
      const tf2::Vector3 point = sensor_to_map * tf2::Vector3(*x, *y, *z);
      if (point.x() < pointcloud_min_x_ || point.x() > pointcloud_max_x_ ||
        point.y() < pointcloud_min_y_ || point.y() > pointcloud_max_y_ ||
        point.z() < pointcloud_min_z_ || point.z() > pointcloud_max_z_)
      {
        continue;
      }
      endpoints.push_back(
        static_cast<float>(point.x()),
        static_cast<float>(point.y()),
        static_cast<float>(point.z()));
    }
  } catch (const std::runtime_error & error) {
    RCLCPP_WARN(logger_, "Sonar PointCloud2 is missing float x/y/z fields: %s", error.what());
    return;
  }

  if (endpoints.size() == 0U) {
    return;
  }

  std::unique_lock<std::shared_mutex> lock(octomap_mutex_);
  if (!mapper_ || !mapper_->insertPointCloud(sensor_origin, endpoints)) {
    return;
  }
  ++map_revision_;
  publishOccupiedCells(cloud->header.stamp);
}

void OctomapPlanner3D::publishOccupiedCells(const builtin_interfaces::msg::Time & message_stamp)
{
  if (!occupied_cells_publisher_ || !occupied_cells_publisher_->is_activated() ||
    occupied_cells_publisher_->get_subscription_count() +
    occupied_cells_publisher_->get_intra_process_subscription_count() == 0U)
  {
    return;
  }

  builtin_interfaces::msg::Time stamp = message_stamp;
  std::int64_t stamp_ns = rclcpp::Time(stamp).nanoseconds();
  if (stamp_ns <= 0) {
    stamp = clock_->now();
    stamp_ns = rclcpp::Time(stamp).nanoseconds();
  }
  const std::int64_t minimum_interval_ns = static_cast<std::int64_t>(
    visualization_interval_ * 1'000'000'000.0);
  if (minimum_interval_ns > 0 && last_visualization_stamp_ns_ > 0 &&
    stamp_ns >= last_visualization_stamp_ns_ &&
    stamp_ns - last_visualization_stamp_ns_ < minimum_interval_ns)
  {
    return;
  }

  const auto & tree = mapper_->tree();
  visualization_msgs::msg::MarkerArray markers;
  markers.markers.resize(tree.getTreeDepth() + 1U);
  for (std::size_t depth = 0; depth < markers.markers.size(); ++depth) {
    auto & marker = markers.markers[depth];
    marker.header.frame_id = map_frame_;
    marker.header.stamp = stamp;
    marker.ns = "map";
    marker.id = static_cast<int>(depth);
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = tree.getNodeSize(static_cast<unsigned int>(depth));
    marker.scale.y = marker.scale.x;
    marker.scale.z = marker.scale.x;
    marker.color.b = 1.0F;
    marker.color.a = 1.0F;
  }

  const double visualization_height = std::max(
    visualization_max_z_ - visualization_min_z_, std::numeric_limits<double>::epsilon());
  for (auto leaf = tree.begin_leafs(), end = tree.end_leafs(); leaf != end; ++leaf) {
    if (!tree.isNodeOccupied(*leaf)) {
      continue;
    }
    const double half_size = leaf.getSize() * 0.5;
    if (leaf.getZ() + half_size <= visualization_min_z_ ||
      leaf.getZ() - half_size >= visualization_max_z_)
    {
      continue;
    }
    auto & marker = markers.markers[leaf.getDepth()];
    geometry_msgs::msg::Point center;
    center.x = leaf.getX();
    center.y = leaf.getY();
    center.z = leaf.getZ();
    marker.points.push_back(center);
    std_msgs::msg::ColorRGBA color;
    const double height = std::clamp(
      (leaf.getZ() - visualization_min_z_) / visualization_height, 0.0, 1.0);
    color.r = static_cast<float>(height);
    color.g = static_cast<float>(1.0 - std::abs(2.0 * height - 1.0));
    color.b = static_cast<float>(1.0 - height);
    color.a = 1.0F;
    marker.colors.push_back(color);
  }
  for (auto & marker : markers.markers) {
    if (!marker.points.empty()) {
      marker.action = visualization_msgs::msg::Marker::ADD;
    }
  }
  occupied_cells_publisher_->publish(markers);
  last_visualization_stamp_ns_ = stamp_ns;
}

nav_msgs::msg::Path OctomapPlanner3D::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  std::shared_lock<std::shared_mutex> lock(octomap_mutex_);
  if (!mapper_ || map_revision_ == 0U) {
    throw nav2_core::NoValidPathCouldBeFound(
            "No OctoMap data has arrived yet; check /sonar/points and its map-frame transform");
  }
  if (map_frame_ != start.header.frame_id) {
    throw nav2_core::NoValidPathCouldBeFound(
            "OctoMap frame '" + map_frame_ + "' does not match planner frame '" +
            start.header.frame_id + "'");
  }

  const Eigen::Vector3d start_point(
    start.pose.position.x, start.pose.position.y, start.pose.position.z);
  const Eigen::Vector3d goal_point(
    goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
  const auto points = planner_->plan(
    mapper_->tree(), map_revision_, start_point, goal_point, cancel_checker);
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
