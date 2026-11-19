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

#include "nav_msgs/msg/odometry.hpp"
#include "pressure_driver/ms5837.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace pressure_driver
{

/// Depth below the surface in metres, positive downwards.
double depth_from_pressure(double pressure_pa, double surface_pressure_pa, double fluid_density_kg_m3);

/// Opens the Bar30 on /dev/i2c-<i2c_bus> and loads its calibration.
/// @throws std::system_error, std::runtime_error as Ms5837 does
std::unique_ptr<Ms5837> open_bar30(int i2c_bus);

/**
 * @brief Publishes the ROV's depth from a Blue Robotics Bar30 as the z position of an odometry message.
 *
 * Configuring opens the sensor; activating starts reading it at `rate_hz`. A failed read publishes
 * nothing, so a stale topic means the sensor is not answering.
 *
 * Parameters:
 *  - `i2c_bus` (int): I2C bus number the sensor is wired to
 *  - `rate_hz` (double): publish rate, at most about 20 Hz
 *  - `fluid_density` (double): water density in kg/m^3
 *  - `surface_pressure_pa` (double): air pressure at the surface in Pa; depth is measured from it
 *  - `depth_variance` (double): variance of the published z in m^2
 *  - `frame_id`, `child_frame_id` (string): frames of the published odometry
 *
 * Publishes:
 *  - `depth` (nav_msgs/Odometry): z = -depth. Only z and its variance are filled in.
 */
class Bar30Node : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using OpenSensor = std::function<std::unique_ptr<Ms5837>(int i2c_bus)>;

  /// `open_sensor` opens the sensor at configure; tests pass one that opens a simulated sensor.
  explicit Bar30Node(const rclcpp::NodeOptions & options, OpenSensor open_sensor = open_bar30);

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void publish_depth();
  void release_resources();

  OpenSensor open_sensor_;

  // Parameters
  double rate_hz_{10.0};
  double fluid_density_{997.0};
  double surface_pressure_pa_{101325.0};
  double depth_variance_{0.0004};
  std::string frame_id_;
  std::string child_frame_id_;

  std::unique_ptr<Ms5837> sensor_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr depth_pub_;
};

}  // namespace pressure_driver
