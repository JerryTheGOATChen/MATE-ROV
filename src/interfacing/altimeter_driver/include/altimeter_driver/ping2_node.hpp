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
#include <optional>
#include <string>

#include "altimeter_driver/ping2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/range.hpp"

namespace altimeter_driver
{

/// A reading converted to metres along the beam.
struct AltitudeRange
{
  double range;
  double min_range;
  double max_range;
};

/// Converts a reading; empty if the device's confidence in it is below `min_confidence` percent.
std::optional<AltitudeRange> range_from_distance(const Ping2Distance & reading, int min_confidence);

/// Opens the Ping2 on a serial port and checks that it answers.
/// @throws std::invalid_argument, std::system_error, std::runtime_error as SerialPort and Ping2 do
std::unique_ptr<Ping2> open_ping2(const std::string & port, int baudrate);

/**
 * @brief Publishes the distance from a Blue Robotics Ping2 to the floor.
 *
 * Configuring opens the device and writes its speed of sound and ping interval; activating starts
 * requesting readings at `rate_hz`. A missing or low-confidence reading publishes nothing, so a stale
 * topic means the altitude is unknown.
 *
 * Parameters:
 *  - `port` (string): serial device the Ping2 is on
 *  - `baudrate` (int): serial baud rate
 *  - `rate_hz` (double): ping and publish rate
 *  - `speed_of_sound` (double): speed of sound in the water in m/s
 *  - `min_confidence` (int): readings the device rates below this percentage are dropped
 *  - `field_of_view` (double): beam width in radians
 *  - `frame_id` (string): frame of the transducer
 *
 * Publishes:
 *  - `altitude` (sensor_msgs/Range): distance along the beam to the floor
 */
class Ping2Node : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using OpenDevice = std::function<std::unique_ptr<Ping2>(const std::string & port, int baudrate)>;

  /// `open_device` opens the device at configure; tests pass one that opens a simulated device.
  explicit Ping2Node(const rclcpp::NodeOptions & options, OpenDevice open_device = open_ping2);

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void publish_altitude();
  void release_resources();

  OpenDevice open_device_;

  // Parameters
  double rate_hz_{10.0};
  int min_confidence_{80};
  double field_of_view_{0.0};
  std::string frame_id_;

  std::unique_ptr<Ping2> device_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Range>::SharedPtr altitude_pub_;
};

}  // namespace altimeter_driver
