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

#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <sensor_msgs/msg/range.hpp>

#include "altimeter_driver/ping2_node.hpp"
#include "fakes/fake_ping2.hpp"
#include "test_fixtures/test_executor_fixture.hpp"
#include "test_nodes/subscriber_test_node.hpp"

using altimeter_driver::Ping2;
using altimeter_driver::Ping2Distance;
using altimeter_driver::Ping2Node;
using altimeter_driver::range_from_distance;
using altimeter_driver::test::FakePing2Stream;
using altimeter_driver::test::SimulatedPing2;
using Catch::Matchers::WithinAbs;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;
using sensor_msgs::msg::Range;
using wato::test::SubscriberTestNode;
using wato::test::TestExecutorFixture;

namespace
{

// A namespace of its own, so the test cannot interfere with other tests or a running stack
constexpr char kNamespace[] = "/ping2_test";
constexpr char kAltitudeTopic[] = "/ping2_test/altitude";
constexpr auto kMessageTimeout = std::chrono::seconds(2);

std::shared_ptr<Ping2Node> make_node(
  std::shared_ptr<SimulatedPing2> device,
  std::string * opened_port = nullptr,
  std::vector<rclcpp::Parameter> overrides = {})
{
  // The test's own overrides come last, so they win
  std::vector<rclcpp::Parameter> parameters{{"port", "/dev/ttyPING"}, {"rate_hz", 20.0}};
  parameters.insert(parameters.end(), overrides.begin(), overrides.end());
  rclcpp::NodeOptions options;
  options.parameter_overrides(parameters);
  options.arguments({"--ros-args", "-r", std::string("__ns:=") + kNamespace});
  return std::make_shared<Ping2Node>(options, [device, opened_port](const std::string & port, int) {
    if (opened_port != nullptr) {
      *opened_port = port;
    }
    return std::make_unique<Ping2>(std::make_unique<FakePing2Stream>(device), std::chrono::milliseconds(50));
  });
}

}  // namespace

TEST_CASE("range_from_distance converts to metres and drops unconfident readings", "[ping2]")
{
  const Ping2Distance reading{3210, 80, 300, 9700};
  const auto range = range_from_distance(reading, 80);
  REQUIRE(range.has_value());
  REQUIRE_THAT(range->range, WithinAbs(3.21, 1e-9));
  REQUIRE_THAT(range->min_range, WithinAbs(0.3, 1e-9));
  REQUIRE_THAT(range->max_range, WithinAbs(10.0, 1e-9));

  REQUIRE_FALSE(range_from_distance(reading, 81).has_value());
}

TEST_CASE_METHOD(TestExecutorFixture, "Ping2Node publishes the altitude", "[ping2]")
{
  auto device = std::make_shared<SimulatedPing2>();
  std::string opened_port;
  auto node = make_node(device, &opened_port);
  auto altitude_sub = std::make_shared<SubscriberTestNode<Range>>(kAltitudeTopic, "altitude_sub");
  add_node(node);
  add_node(altitude_sub);
  start_spinning();

  REQUIRE(node->trigger_transition(Transition::TRANSITION_CONFIGURE).id() == State::PRIMARY_STATE_INACTIVE);
  REQUIRE(opened_port == "/dev/ttyPING");
  REQUIRE(device->speed_of_sound_mm_s == 1500000);
  REQUIRE(device->ping_interval_ms == 50);  // 20 Hz
  REQUIRE(node->trigger_transition(Transition::TRANSITION_ACTIVATE).id() == State::PRIMARY_STATE_ACTIVE);
  REQUIRE(altitude_sub->wait_for_publishers(1));

  SECTION("It publishes the reading along the beam")
  {
    auto future = altitude_sub->expect_next_message();
    REQUIRE(future.wait_for(kMessageTimeout) == std::future_status::ready);
    const Range msg = future.get();
    REQUIRE_THAT(msg.range, WithinAbs(3.21, 1e-6));
    REQUIRE_THAT(msg.min_range, WithinAbs(0.3, 1e-6));
    REQUIRE_THAT(msg.max_range, WithinAbs(10.0, 1e-6));
    REQUIRE_THAT(msg.field_of_view, WithinAbs(25.0 * M_PI / 180.0, 1e-6));
    REQUIRE(msg.radiation_type == Range::ULTRASOUND);
    REQUIRE(msg.header.frame_id == "ping2_link");
  }

  SECTION("A low-confidence reading publishes nothing")
  {
    device->confidence = 30;
    // Let any reading already in flight arrive, then expect silence
    auto in_flight = altitude_sub->expect_next_message();
    in_flight.wait_for(std::chrono::milliseconds(200));
    auto future = altitude_sub->expect_next_message();
    REQUIRE(future.wait_for(std::chrono::milliseconds(500)) == std::future_status::timeout);
  }

  SECTION("Cleaning up releases the device")
  {
    REQUIRE(node->trigger_transition(Transition::TRANSITION_DEACTIVATE).id() == State::PRIMARY_STATE_INACTIVE);
    REQUIRE(node->trigger_transition(Transition::TRANSITION_CLEANUP).id() == State::PRIMARY_STATE_UNCONFIGURED);
    REQUIRE(device->closed);
  }
}

TEST_CASE_METHOD(TestExecutorFixture, "Ping2Node fails to configure", "[ping2]")
{
  auto device = std::make_shared<SimulatedPing2>();

  SECTION("When the device does not answer")
  {
    device->answers = false;
    auto node = make_node(device);
    add_node(node);
    REQUIRE(node->trigger_transition(Transition::TRANSITION_CONFIGURE).id() == State::PRIMARY_STATE_UNCONFIGURED);
  }

  SECTION("When the device keeps its own settings, and it releases the device")
  {
    device->keeps_own_settings = true;
    auto node = make_node(device);
    add_node(node);
    REQUIRE(node->trigger_transition(Transition::TRANSITION_CONFIGURE).id() == State::PRIMARY_STATE_UNCONFIGURED);
    REQUIRE(device->closed);
  }

  SECTION("With a rate too low for the ping interval")
  {
    auto node = make_node(device, nullptr, {{"rate_hz", 0.01}});
    add_node(node);
    REQUIRE(node->trigger_transition(Transition::TRANSITION_CONFIGURE).id() == State::PRIMARY_STATE_UNCONFIGURED);
  }
}
