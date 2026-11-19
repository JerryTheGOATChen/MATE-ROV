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

#include "pressure_driver/bar30_node.hpp"

#include <chrono>
#include <exception>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp_components/register_node_macro.hpp"

namespace pressure_driver
{
namespace
{

constexpr double kGravity = 9.80665;  // m/s^2
constexpr std::size_t kZVarianceIndex = 14;  // z variance in a row-major 6x6 pose covariance
constexpr int kWarningThrottleMs = 5000;

}  // namespace

double depth_from_pressure(double pressure_pa, double surface_pressure_pa, double fluid_density_kg_m3)
{
  return (pressure_pa - surface_pressure_pa) / (fluid_density_kg_m3 * kGravity);
}

std::unique_ptr<Ms5837> open_bar30(int i2c_bus)
{
  return std::make_unique<Ms5837>(std::make_unique<LinuxI2cDevice>(i2c_bus, Ms5837::kAddress));
}

Bar30Node::Bar30Node(const rclcpp::NodeOptions & options, OpenSensor open_sensor)
: LifecycleNode("bar30_node", options)
, open_sensor_(std::move(open_sensor))
{
  declare_parameter<int>("i2c_bus", 1);
  declare_parameter<double>("rate_hz", 10.0);
  declare_parameter<double>("fluid_density", 997.0);
  declare_parameter<double>("surface_pressure_pa", 101325.0);
  declare_parameter<double>("depth_variance", 0.0004);
  declare_parameter<std::string>("frame_id", "odom");
  declare_parameter<std::string>("child_frame_id", "base_link");
}

Bar30Node::CallbackReturn Bar30Node::on_configure(const rclcpp_lifecycle::State &)
{
  const int i2c_bus = get_parameter("i2c_bus").as_int();
  rate_hz_ = get_parameter("rate_hz").as_double();
  fluid_density_ = get_parameter("fluid_density").as_double();
  surface_pressure_pa_ = get_parameter("surface_pressure_pa").as_double();
  depth_variance_ = get_parameter("depth_variance").as_double();
  frame_id_ = get_parameter("frame_id").as_string();
  child_frame_id_ = get_parameter("child_frame_id").as_string();

  if (rate_hz_ <= 0.0 || fluid_density_ <= 0.0 || depth_variance_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "rate_hz, fluid_density and depth_variance must be positive");
    return CallbackReturn::FAILURE;
  }

  try {
    sensor_ = open_sensor_(i2c_bus);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Could not open the Bar30 on I2C bus %d: %s", i2c_bus, e.what());
    return CallbackReturn::FAILURE;
  }

  depth_pub_ = create_publisher<nav_msgs::msg::Odometry>("depth", 10);
  RCLCPP_INFO(get_logger(), "Configured the Bar30 on I2C bus %d", i2c_bus);
  return CallbackReturn::SUCCESS;
}

Bar30Node::CallbackReturn Bar30Node::on_activate(const rclcpp_lifecycle::State &)
{
  depth_pub_->on_activate();
  timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz_), [this]() { publish_depth(); });
  return CallbackReturn::SUCCESS;
}

Bar30Node::CallbackReturn Bar30Node::on_deactivate(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  depth_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

Bar30Node::CallbackReturn Bar30Node::on_cleanup(const rclcpp_lifecycle::State &)
{
  release_resources();
  return CallbackReturn::SUCCESS;
}

Bar30Node::CallbackReturn Bar30Node::on_shutdown(const rclcpp_lifecycle::State &)
{
  release_resources();
  return CallbackReturn::SUCCESS;
}

void Bar30Node::publish_depth()
{
  Ms5837Reading reading{};
  try {
    reading = sensor_->read();
  } catch (const std::exception & e) {
    // Publish nothing, so that consumers see a stale topic and not a wrong depth
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarningThrottleMs, "Bar30 read failed: %s", e.what());
    return;
  }

  nav_msgs::msg::Odometry msg;
  msg.header.stamp = now();
  msg.header.frame_id = frame_id_;
  msg.child_frame_id = child_frame_id_;
  msg.pose.pose.position.z = -depth_from_pressure(reading.pressure_pa, surface_pressure_pa_, fluid_density_);
  msg.pose.covariance[kZVarianceIndex] = depth_variance_;
  depth_pub_->publish(msg);
}

void Bar30Node::release_resources()
{
  timer_.reset();
  depth_pub_.reset();
  sensor_.reset();
}

}  // namespace pressure_driver

RCLCPP_COMPONENTS_REGISTER_NODE(pressure_driver::Bar30Node)
