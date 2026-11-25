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

#include "altimeter_driver/ping2_node.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "rclcpp_components/register_node_macro.hpp"

namespace altimeter_driver
{
namespace
{

constexpr double kMmPerM = 1000.0;
constexpr double kMsPerS = 1000.0;
constexpr int64_t kMaxPingIntervalMs = 65535;  // a 16-bit field
constexpr double kPing2BeamWidthRad = 25.0 * M_PI / 180.0;
constexpr int kWarningThrottleMs = 5000;

}  // namespace

std::optional<AltitudeRange> range_from_distance(const Ping2Distance & reading, int min_confidence)
{
  if (reading.confidence < min_confidence) {
    return std::nullopt;
  }
  return AltitudeRange{
    reading.distance_mm / kMmPerM,
    reading.scan_start_mm / kMmPerM,
    (static_cast<double>(reading.scan_start_mm) + reading.scan_length_mm) / kMmPerM};
}

std::unique_ptr<Ping2> open_ping2(const std::string & port, int baudrate)
{
  return std::make_unique<Ping2>(std::make_unique<SerialPort>(port, baudrate));
}

Ping2Node::Ping2Node(const rclcpp::NodeOptions & options, OpenDevice open_device)
: LifecycleNode("ping2_node", options)
, open_device_(std::move(open_device))
{
  declare_parameter<std::string>("port", "/dev/ttyUSB0");
  declare_parameter<int>("baudrate", 115200);
  declare_parameter<double>("rate_hz", 10.0);
  declare_parameter<double>("speed_of_sound", 1500.0);
  declare_parameter<int>("min_confidence", 80);
  declare_parameter<double>("field_of_view", kPing2BeamWidthRad);
  declare_parameter<std::string>("frame_id", "ping2_link");
}

Ping2Node::CallbackReturn Ping2Node::on_configure(const rclcpp_lifecycle::State &)
{
  const std::string port = get_parameter("port").as_string();
  const int baudrate = get_parameter("baudrate").as_int();
  const double speed_of_sound = get_parameter("speed_of_sound").as_double();
  rate_hz_ = get_parameter("rate_hz").as_double();
  min_confidence_ = get_parameter("min_confidence").as_int();
  field_of_view_ = get_parameter("field_of_view").as_double();
  frame_id_ = get_parameter("frame_id").as_string();

  const int64_t ping_interval_ms = rate_hz_ > 0.0 ? std::llround(kMsPerS / rate_hz_) : 0;
  if (ping_interval_ms < 1 || ping_interval_ms > kMaxPingIntervalMs || speed_of_sound <= 0.0) {
    RCLCPP_ERROR(
      get_logger(), "rate_hz must give a ping interval of 1 to 65535 ms, and speed_of_sound must be positive");
    return CallbackReturn::FAILURE;
  }

  try {
    device_ = open_device_(port, baudrate);
    device_->configure(
      static_cast<uint32_t>(std::lround(speed_of_sound * kMmPerM)), static_cast<uint16_t>(ping_interval_ms));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Could not set up the Ping2 on %s: %s", port.c_str(), e.what());
    release_resources();
    return CallbackReturn::FAILURE;
  }

  altitude_pub_ = create_publisher<sensor_msgs::msg::Range>("altitude", 10);
  RCLCPP_INFO(get_logger(), "Configured the Ping2 on %s", port.c_str());
  return CallbackReturn::SUCCESS;
}

Ping2Node::CallbackReturn Ping2Node::on_activate(const rclcpp_lifecycle::State &)
{
  altitude_pub_->on_activate();
  timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz_), [this]() { publish_altitude(); });
  return CallbackReturn::SUCCESS;
}

Ping2Node::CallbackReturn Ping2Node::on_deactivate(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  altitude_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

Ping2Node::CallbackReturn Ping2Node::on_cleanup(const rclcpp_lifecycle::State &)
{
  release_resources();
  return CallbackReturn::SUCCESS;
}

Ping2Node::CallbackReturn Ping2Node::on_shutdown(const rclcpp_lifecycle::State &)
{
  release_resources();
  return CallbackReturn::SUCCESS;
}

void Ping2Node::publish_altitude()
{
  std::optional<Ping2Distance> reading;
  try {
    reading = device_->get_distance();
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarningThrottleMs, "Ping2 read failed: %s", e.what());
    return;
  }

  // Publish nothing for a missing or low-confidence reading, so that consumers see a stale topic and
  // not a wrong altitude
  if (!reading) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarningThrottleMs, "No reply from the Ping2");
    return;
  }
  const auto range = range_from_distance(*reading, min_confidence_);
  if (!range) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kWarningThrottleMs, "No confident Ping2 reading (%u%%)", reading->confidence);
    return;
  }

  sensor_msgs::msg::Range msg;
  msg.header.stamp = now();
  msg.header.frame_id = frame_id_;
  msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  msg.field_of_view = static_cast<float>(field_of_view_);
  msg.range = static_cast<float>(range->range);
  msg.min_range = static_cast<float>(range->min_range);
  msg.max_range = static_cast<float>(range->max_range);
  altitude_pub_->publish(msg);
}

void Ping2Node::release_resources()
{
  timer_.reset();
  altitude_pub_.reset();
  device_.reset();
}

}  // namespace altimeter_driver

RCLCPP_COMPONENTS_REGISTER_NODE(altimeter_driver::Ping2Node)
