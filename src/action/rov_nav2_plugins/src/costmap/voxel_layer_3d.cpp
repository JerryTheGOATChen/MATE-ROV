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

#include "rov_nav2_plugins/costmap/voxel_layer_3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <Eigen/Geometry>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/exceptions.h"

namespace rov_nav2_plugins
{

void VoxelLayer3D::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("VoxelLayer3D: unable to lock the parent node");
  }

  enabled_ = nav2_util::declare_or_get_parameter(node, name_ + ".enabled", true);
  const auto topic = nav2_util::declare_or_get_parameter(node, name_ + ".topic", std::string("/sonar/points"));
  min_z_ = nav2_util::declare_or_get_parameter(node, name_ + ".min_z", -4.4);
  max_z_ = nav2_util::declare_or_get_parameter(node, name_ + ".max_z", 0.0);
  obstacle_range_ = nav2_util::declare_or_get_parameter(node, name_ + ".obstacle_range", 8.0);
  voxel_decay_ = nav2_util::declare_or_get_parameter(node, name_ + ".voxel_decay", 60.0);
  robot_half_height_ = nav2_util::declare_or_get_parameter(node, name_ + ".robot_half_height", 0.2);
  inflation_radius_ = nav2_util::declare_or_get_parameter(node, name_ + ".inflation_radius", 1.0);
  cost_scaling_factor_ = nav2_util::declare_or_get_parameter(node, name_ + ".cost_scaling_factor", 3.0);
  transform_tolerance_ = nav2_util::declare_or_get_parameter(node, name_ + ".transform_tolerance", 0.5);
  const bool publish_voxel_costs = nav2_util::declare_or_get_parameter(node, name_ + ".publish_voxel_costs", false);

  if (max_z_ <= min_z_) {
    throw std::invalid_argument(name_ + ": max_z must be above min_z");
  }
  if (
    obstacle_range_ <= 0.0 || voxel_decay_ <= 0.0 || robot_half_height_ <= 0.0 || inflation_radius_ <= 0.0 ||
    cost_scaling_factor_ <= 0.0 || transform_tolerance_ < 0.0)
  {
    throw std::invalid_argument(
      name_ + ": obstacle range, voxel decay, half-height, inflation radius and cost scaling factor must be positive");
  }

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic,
    rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) { on_cloud(std::move(cloud)); },
    options);
  if (publish_voxel_costs) {
    voxel_costs_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(name_ + "/voxel_costs", 1);
  }

  matchSize();
  current_ = true;

  RCLCPP_INFO(
    logger_,
    "Configured %s: topic=%s, z from %.2f to %.2f m, voxel_decay=%.1f s, inflation_radius=%.2f m",
    name_.c_str(),
    topic.c_str(),
    min_z_,
    max_z_,
    voxel_decay_,
    inflation_radius_);
}

void VoxelLayer3D::activate()
{
  if (voxel_costs_pub_) {
    voxel_costs_pub_->on_activate();
  }
}

void VoxelLayer3D::deactivate()
{
  if (voxel_costs_pub_) {
    voxel_costs_pub_->on_deactivate();
  }
}

void VoxelLayer3D::reset()
{
  {
    std::lock_guard<std::mutex> lock(clouds_mutex_);
    clouds_.clear();
  }
  if (grid_) {
    grid_->clear();
  }
  grid_changed_ = true;
  current_ = true;
}

void VoxelLayer3D::matchSize()
{
  // The grid's columns are the master costmap's cells, so the grid covers the costmap's window
  const auto * master = layered_costmap_->getCostmap();
  GridGeometry geometry;
  geometry.resolution = master->getResolution();
  geometry.origin = {master->getOriginX(), master->getOriginY(), min_z_};
  geometry.size = {
    static_cast<int>(master->getSizeInCellsX()),
    static_cast<int>(master->getSizeInCellsY()),
    std::max(1, static_cast<int>(std::ceil((max_z_ - min_z_) / geometry.resolution - 1e-9)))};

  // The costmap has no size until it is configured. Until the first update computes the costs, the planner sees
  // the grid as it starts: every voxel unknown and free.
  std::shared_ptr<const CostVolume> volume;
  if (geometry.resolution > 0.0 && geometry.size.x() > 0 && geometry.size.y() > 0) {
    grid_.emplace(geometry);
    volume = std::make_shared<const CostVolume>(
      geometry, std::vector<std::uint8_t>(geometry.count(), kFreeCost), std::vector<bool>(geometry.count(), false));
  } else {
    grid_.reset();
  }
  grid_changed_ = true;
  std::lock_guard<std::mutex> lock(volume_mutex_);
  volume_ = volume;
}

bool VoxelLayer3D::isClearable()
{
  return true;
}

void VoxelLayer3D::updateBounds(
  double /*robot_x*/,
  double /*robot_y*/,
  double /*robot_yaw*/,
  double * /*min_x*/,
  double * /*min_y*/,
  double * /*max_x*/,
  double * /*max_y*/)
{
  // The bounds are left alone, because the layer writes nothing into the 2D costmap
  if (!enabled_ || !grid_) {
    return;
  }

  // Insert the clouds received since the last update. A cloud can arrive before its transform, so one whose
  // transform is missing waits for the next update, up to transform_tolerance.
  std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> clouds;
  {
    std::lock_guard<std::mutex> lock(clouds_mutex_);
    clouds.swap(clouds_);
  }
  const double now = clock_->now().seconds();
  std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> waiting;
  for (const auto & cloud : clouds) {
    if (insert_cloud(*cloud)) {
      grid_changed_ = true;
    } else if (now - rclcpp::Time(cloud->header.stamp).seconds() < transform_tolerance_) {
      waiting.push_back(cloud);
    } else {
      RCLCPP_WARN_THROTTLE(
        logger_,
        *clock_,
        5000,
        "%s: dropped a cloud with no transform from %s to %s",
        name_.c_str(),
        cloud->header.frame_id.c_str(),
        layered_costmap_->getGlobalFrameID().c_str());
    }
  }
  if (!waiting.empty()) {
    std::lock_guard<std::mutex> lock(clouds_mutex_);
    clouds_.insert(clouds_.begin(), waiting.begin(), waiting.end());
  }

  if (grid_->decay(now, voxel_decay_) > 0) {
    grid_changed_ = true;
  }
  if (!grid_changed_) {
    return;
  }

  InflationConfig config;
  config.inscribed_radius = layered_costmap_->getInscribedRadius();
  config.half_height = robot_half_height_;
  // As in Nav2's InflationLayer, cost never reaches zero inside the vehicle
  config.inflation_radius = std::max(inflation_radius_, config.inscribed_radius);
  config.cost_scaling_factor = cost_scaling_factor_;
  if (config.inscribed_radius <= 0.0) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000, "%s: waiting for the costmap's footprint or robot_radius to inflate", name_.c_str());
    return;
  }

  auto volume = std::make_shared<const CostVolume>(inflate(*grid_, config));
  {
    std::lock_guard<std::mutex> lock(volume_mutex_);
    volume_ = volume;
  }
  grid_changed_ = false;

  if (voxel_costs_pub_ && voxel_costs_pub_->is_activated() && voxel_costs_pub_->get_subscription_count() > 0) {
    publish_voxel_costs(*volume);
  }
}

void VoxelLayer3D::updateCosts(
  nav2_costmap_2d::Costmap2D & /*master_grid*/, int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
{
  // The costs stay in 3D, for the planner to read from cost_volume(). No 2D projection of them is safe to
  // plan on: an obstacle at one depth either blocks every depth or none.
}

std::shared_ptr<const CostVolume> VoxelLayer3D::cost_volume() const
{
  std::lock_guard<std::mutex> lock(volume_mutex_);
  return volume_;
}

void VoxelLayer3D::on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  std::lock_guard<std::mutex> lock(clouds_mutex_);
  clouds_.push_back(std::move(cloud));
}

bool VoxelLayer3D::insert_cloud(const sensor_msgs::msg::PointCloud2 & cloud)
{
  geometry_msgs::msg::TransformStamped sensor_to_global_msg;
  try {
    sensor_to_global_msg = tf_->lookupTransform(
      layered_costmap_->getGlobalFrameID(), cloud.header.frame_id, rclcpp::Time(cloud.header.stamp));
  } catch (const tf2::TransformException &) {
    return false;
  }
  const auto & translation = sensor_to_global_msg.transform.translation;
  const auto & rotation = sensor_to_global_msg.transform.rotation;
  const Eigen::Isometry3d sensor_to_global = Eigen::Translation3d(translation.x, translation.y, translation.z) *
                                             Eigen::Quaterniond(rotation.w, rotation.x, rotation.y, rotation.z);

  std::vector<Eigen::Vector3d> points;
  points.reserve(static_cast<std::size_t>(cloud.width) * cloud.height);
  try {
    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      points.push_back(sensor_to_global * Eigen::Vector3d(*x, *y, *z));
    }
  } catch (const std::runtime_error & error) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "%s: dropped a cloud: %s", name_.c_str(), error.what());
    return true;
  }

  grid_->insert(sensor_to_global.translation(), points, rclcpp::Time(cloud.header.stamp).seconds(), obstacle_range_);
  return true;
}

void VoxelLayer3D::publish_voxel_costs(const CostVolume & volume)
{
  const GridGeometry & geometry = volume.geometry();
  std::vector<Eigen::Vector3i> voxels;
  Eigen::Vector3i voxel;
  for (voxel.z() = 0; voxel.z() < geometry.size.z(); ++voxel.z()) {
    for (voxel.y() = 0; voxel.y() < geometry.size.y(); ++voxel.y()) {
      for (voxel.x() = 0; voxel.x() < geometry.size.x(); ++voxel.x()) {
        if (volume.cost(voxel) != kFreeCost) {
          voxels.push_back(voxel);
        }
      }
    }
  }

  auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
  cloud->header.frame_id = layered_costmap_->getGlobalFrameID();
  cloud->header.stamp = clock_->now();
  sensor_msgs::PointCloud2Modifier modifier(*cloud);
  modifier.setPointCloud2Fields(
    4,
    "x",
    1,
    sensor_msgs::msg::PointField::FLOAT32,
    "y",
    1,
    sensor_msgs::msg::PointField::FLOAT32,
    "z",
    1,
    sensor_msgs::msg::PointField::FLOAT32,
    "intensity",
    1,
    sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(voxels.size());

  sensor_msgs::PointCloud2Iterator<float> x(*cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(*cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(*cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> intensity(*cloud, "intensity");
  for (const auto & v : voxels) {
    const Eigen::Vector3d center = geometry.center(v);
    *x = static_cast<float>(center.x());
    *y = static_cast<float>(center.y());
    *z = static_cast<float>(center.z());
    *intensity = static_cast<float>(volume.cost(v));
    ++x;
    ++y;
    ++z;
    ++intensity;
  }
  voxel_costs_pub_->publish(std::move(cloud));
}

}  // namespace rov_nav2_plugins

PLUGINLIB_EXPORT_CLASS(rov_nav2_plugins::VoxelLayer3D, nav2_costmap_2d::Layer)
