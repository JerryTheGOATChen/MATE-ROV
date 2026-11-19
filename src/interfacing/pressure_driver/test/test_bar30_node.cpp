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
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "fakes/fake_ms5837.hpp"
#include "pressure_driver/bar30_node.hpp"
#include "test_fixtures/test_executor_fixture.hpp"
#include "test_nodes/subscriber_test_node.hpp"

using Catch::Matchers::WithinAbs;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;
using nav_msgs::msg::Odometry;
using pressure_driver::Bar30Node;
using pressure_driver::compensate;
using pressure_driver::depth_from_pressure;
using pressure_driver::Ms5837;
using pressure_driver::test::datasheet_calibration;
using pressure_driver::test::FakeMs5837Device;
using pressure_driver::test::kDatasheetRawPressure;
using pressure_driver::test::kDatasheetRawTemperature;
using pressure_driver::test::SimulatedMs5837;
using wato::test::SubscriberTestNode;
using wato::test::TestExecutorFixture;

namespace
{

// A namespace of its own, so the test cannot interfere with other tests or a running stack
constexpr char kNamespace[] = "/bar30_test";
constexpr char kDepthTopic[] = "/bar30_test/depth";
constexpr auto kMessageTimeout = std::chrono::seconds(2);

std::shared_ptr<Bar30Node> make_node(
  std::shared_ptr<SimulatedMs5837> sensor, int * opened_bus = nullptr, std::vector<rclcpp::Parameter> overrides = {})
{
  // The test's own overrides come last, so they win
  std::vector<rclcpp::Parameter> parameters{{"i2c_bus", 3}, {"rate_hz", 20.0}};
  parameters.insert(parameters.end(), overrides.begin(), overrides.end());
  rclcpp::NodeOptions options;
  options.parameter_overrides(parameters);
  options.arguments({"--ros-args", "-r", std::string("__ns:=") + kNamespace});
  return std::make_shared<Bar30Node>(options, [sensor, opened_bus](int bus) {
    if (opened_bus != nullptr) {
      *opened_bus = bus;
    }
    return std::make_unique<Ms5837>(std::make_unique<FakeMs5837Device>(sensor));
  });
}

}  // namespace

TEST_CASE("depth_from_pressure measures down from the surface pressure", "[bar30]")
{
  REQUIRE_THAT(depth_from_pressure(101325.0, 101325.0, 997.0), WithinAbs(0.0, 1e-12));
  REQUIRE_THAT(depth_from_pressure(101325.0 + 997.0 * 9.80665 * 2.0, 101325.0, 997.0), WithinAbs(2.0, 1e-9));
}

TEST_CASE_METHOD(TestExecutorFixture, "Bar30Node publishes the depth", "[bar30]")
{
  auto sensor = std::make_shared<SimulatedMs5837>();
  int opened_bus = -1;
  auto node = make_node(sensor, &opened_bus);
  auto depth_sub = std::make_shared<SubscriberTestNode<Odometry>>(kDepthTopic, "depth_sub");
  add_node(node);
  add_node(depth_sub);
  start_spinning();

  REQUIRE(node->trigger_transition(Transition::TRANSITION_CONFIGURE).id() == State::PRIMARY_STATE_INACTIVE);
  REQUIRE(opened_bus == 3);
  REQUIRE(node->trigger_transition(Transition::TRANSITION_ACTIVATE).id() == State::PRIMARY_STATE_ACTIVE);
  REQUIRE(depth_sub->wait_for_publishers(1));

  SECTION("z is the negative depth, with its variance")
  {
    auto future = depth_sub->expect_next_message();
    REQUIRE(future.wait_for(kMessageTimeout) == std::future_status::ready);
    const Odometry msg = future.get();

    const double pressure_pa =
      compensate(datasheet_calibration(), kDatasheetRawPressure, kDatasheetRawTemperature).pressure_pa;
    REQUIRE_THAT(msg.pose.pose.position.z, WithinAbs(-depth_from_pressure(pressure_pa, 101325.0, 997.0), 1e-9));
    REQUIRE(msg.pose.covariance[14] == 0.0004);
    REQUIRE(msg.header.frame_id == "odom");
    REQUIRE(msg.child_frame_id == "base_link");
  }

  SECTION("A failed read publishes nothing")
  {
    sensor->fail_transfers = true;
    // Let any reading already in flight arrive, then expect silence
    auto in_flight = depth_sub->expect_next_message();
    in_flight.wait_for(std::chrono::milliseconds(200));
    auto future = depth_sub->expect_next_message();
    REQUIRE(future.wait_for(std::chrono::milliseconds(500)) == std::future_status::timeout);
  }

  SECTION("Cleaning up releases the sensor")
  {
    REQUIRE(node->trigger_transition(Transition::TRANSITION_DEACTIVATE).id() == State::PRIMARY_STATE_INACTIVE);
    REQUIRE(node->trigger_transition(Transition::TRANSITION_CLEANUP).id() == State::PRIMARY_STATE_UNCONFIGURED);
    REQUIRE(sensor->closed);
  }
}

TEST_CASE_METHOD(TestExecutorFixture, "Bar30Node fails to configure", "[bar30]")
{
  auto sensor = std::make_shared<SimulatedMs5837>();

  SECTION("When the sensor does not answer")
  {
    sensor->fail_transfers = true;
    auto node = make_node(sensor);
    add_node(node);
    REQUIRE(node->trigger_transition(Transition::TRANSITION_CONFIGURE).id() == State::PRIMARY_STATE_UNCONFIGURED);
  }

  SECTION("When the calibration fails its checksum")
  {
    sensor->calibration[1] ^= 0x0010;
    auto node = make_node(sensor);
    add_node(node);
    REQUIRE(node->trigger_transition(Transition::TRANSITION_CONFIGURE).id() == State::PRIMARY_STATE_UNCONFIGURED);
  }

  SECTION("With a non-positive rate")
  {
    auto node = make_node(sensor, nullptr, {{"rate_hz", 0.0}});
    add_node(node);
    REQUIRE(node->trigger_transition(Transition::TRANSITION_CONFIGURE).id() == State::PRIMARY_STATE_UNCONFIGURED);
  }
}
