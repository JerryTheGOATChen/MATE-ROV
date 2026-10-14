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
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "rov_kinematic_sim/kinematic_sim_node.hpp"
#include "test_fixtures/test_executor_fixture.hpp"
#include "test_nodes/publisher_test_node.hpp"
#include "test_nodes/subscriber_test_node.hpp"

using Catch::Matchers::WithinAbs;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;
using rov_kinematic_sim::KinematicSimNode;
using wato::test::PublisherTestNode;
using wato::test::SubscriberTestNode;
using wato::test::TestExecutorFixture;

using Odometry = nav_msgs::msg::Odometry;
using TwistStamped = geometry_msgs::msg::TwistStamped;

namespace
{

// Topics and frames unique to this test, so it cannot interfere with other tests or a running stack
constexpr char kNamespace[] = "/kinematic_sim_test";
constexpr char kCmdVelTopic[] = "/kinematic_sim_test/cmd_vel";
constexpr char kOdomTopic[] = "/kinematic_sim_test/odom";
constexpr char kOdomFrame[] = "kinematic_sim_test_odom";
constexpr char kBaseFrame[] = "kinematic_sim_test_base_link";

constexpr auto kMessageTimeout = std::chrono::seconds(2);
constexpr int kMaxUpdates = 100;  // 2 s of odometry at 50 Hz

std::shared_ptr<KinematicSimNode> make_sim(std::vector<rclcpp::Parameter> overrides = {})
{
  overrides.emplace_back("odom_frame", kOdomFrame);
  overrides.emplace_back("base_frame", kBaseFrame);
  overrides.emplace_back("response_time_s", 0.0);  // respond instantly, so motion is easy to predict
  overrides.emplace_back("cmd_vel_timeout_s", 0.2);

  rclcpp::NodeOptions options;
  options.parameter_overrides(overrides);
  options.arguments({"--ros-args", "-r", std::string("__ns:=") + kNamespace});
  return std::make_shared<KinematicSimNode>(options);
}

TwistStamped command(double surge, double heave = 0.0, const std::string & frame = kBaseFrame)
{
  TwistStamped msg;
  msg.header.frame_id = frame;
  msg.twist.linear.x = surge;
  msg.twist.linear.z = heave;
  return msg;
}

template <typename MessageType>
MessageType next_message(SubscriberTestNode<MessageType> & sub)
{
  auto future = sub.expect_next_message();
  REQUIRE(future.wait_for(kMessageTimeout) == std::future_status::ready);
  return future.get();
}

}  // namespace

TEST_CASE_METHOD(TestExecutorFixture, "KinematicSimNode simulates the vehicle", "[kinematic_sim]")
{
  auto sim = make_sim({{"initial_pose", std::vector<double>{1.0, 2.0, -3.0, 0.5}}});
  auto cmd_pub = std::make_shared<PublisherTestNode<TwistStamped>>(kCmdVelTopic, "cmd_pub");
  auto odom_sub = std::make_shared<SubscriberTestNode<Odometry>>(kOdomTopic, "odom_sub");
  add_node(sim);
  add_node(cmd_pub);
  add_node(odom_sub);
  start_spinning();

  REQUIRE(sim->trigger_transition(Transition::TRANSITION_CONFIGURE).id() == State::PRIMARY_STATE_INACTIVE);
  REQUIRE(sim->trigger_transition(Transition::TRANSITION_ACTIVATE).id() == State::PRIMARY_STATE_ACTIVE);
  REQUIRE(cmd_pub->wait_for_subscribers(1));
  REQUIRE(odom_sub->wait_for_publishers(1));

  SECTION("It starts at the initial pose")
  {
    const Odometry odom = next_message(*odom_sub);
    REQUIRE(odom.header.frame_id == kOdomFrame);
    REQUIRE(odom.child_frame_id == kBaseFrame);
    REQUIRE_THAT(odom.pose.pose.position.x, WithinAbs(1.0, 1e-9));
    REQUIRE_THAT(odom.pose.pose.position.y, WithinAbs(2.0, 1e-9));
    REQUIRE_THAT(odom.pose.pose.position.z, WithinAbs(-3.0, 1e-9));
  }

  SECTION("It follows commands and stops when they stop")
  {
    double surge = 0.0;
    for (int i = 0; i < kMaxUpdates && surge < 0.5 - 1e-6; ++i) {
      cmd_pub->publish(command(0.5));
      surge = next_message(*odom_sub).twist.twist.linear.x;
    }
    REQUIRE_THAT(surge, WithinAbs(0.5, 1e-6));

    // Without fresh commands, the vehicle stops once cmd_vel_timeout_s passes
    for (int i = 0; i < kMaxUpdates && surge > 0.0; ++i) {
      surge = next_message(*odom_sub).twist.twist.linear.x;
    }
    REQUIRE(surge == 0.0);
  }

  SECTION("It ignores commands in another frame")
  {
    for (int i = 0; i < 25; ++i) {
      cmd_pub->publish(command(0.5, 0.0, "map"));
      REQUIRE(next_message(*odom_sub).twist.twist.linear.x == 0.0);
    }
  }
}

TEST_CASE_METHOD(TestExecutorFixture, "KinematicSimNode floats at the surface", "[kinematic_sim]")
{
  auto sim = make_sim();  // starts at the surface
  auto cmd_pub = std::make_shared<PublisherTestNode<TwistStamped>>(kCmdVelTopic, "cmd_pub");
  auto odom_sub = std::make_shared<SubscriberTestNode<Odometry>>(kOdomTopic, "odom_sub");
  add_node(sim);
  add_node(cmd_pub);
  add_node(odom_sub);
  start_spinning();

  REQUIRE(sim->trigger_transition(Transition::TRANSITION_CONFIGURE).id() == State::PRIMARY_STATE_INACTIVE);
  REQUIRE(sim->trigger_transition(Transition::TRANSITION_ACTIVATE).id() == State::PRIMARY_STATE_ACTIVE);
  REQUIRE(cmd_pub->wait_for_subscribers(1));

  for (int i = 0; i < 25; ++i) {
    cmd_pub->publish(command(0.0, 0.5));
    const Odometry odom = next_message(*odom_sub);
    REQUIRE(odom.pose.pose.position.z == 0.0);
    REQUIRE(odom.twist.twist.linear.z == 0.0);
  }
}

TEST_CASE_METHOD(TestExecutorFixture, "KinematicSimNode broadcasts the odom to base transform", "[kinematic_sim]")
{
  auto sim = make_sim();
  auto tf_sub = std::make_shared<SubscriberTestNode<tf2_msgs::msg::TFMessage>>("/tf", "tf_sub");
  add_node(sim);
  add_node(tf_sub);
  start_spinning();

  REQUIRE(sim->trigger_transition(Transition::TRANSITION_CONFIGURE).id() == State::PRIMARY_STATE_INACTIVE);
  REQUIRE(sim->trigger_transition(Transition::TRANSITION_ACTIVATE).id() == State::PRIMARY_STATE_ACTIVE);
  REQUIRE(tf_sub->wait_for_publishers(1));

  // Other processes may also publish on /tf, so look for this simulator's transform
  bool found = false;
  for (int i = 0; i < kMaxUpdates && !found; ++i) {
    for (const auto & transform : next_message(*tf_sub).transforms) {
      found = found || (transform.header.frame_id == kOdomFrame && transform.child_frame_id == kBaseFrame);
    }
  }
  REQUIRE(found);
}

TEST_CASE_METHOD(TestExecutorFixture, "KinematicSimNode rejects invalid parameters", "[kinematic_sim]")
{
  SECTION("Non-positive update rate")
  {
    auto sim = make_sim({{"update_rate_hz", 0.0}});
    add_node(sim);
    REQUIRE(sim->trigger_transition(Transition::TRANSITION_CONFIGURE).id() == State::PRIMARY_STATE_UNCONFIGURED);
  }

  SECTION("Starting above the surface")
  {
    auto sim = make_sim({{"initial_pose", std::vector<double>{0.0, 0.0, 1.0, 0.0}}});
    add_node(sim);
    REQUIRE(sim->trigger_transition(Transition::TRANSITION_CONFIGURE).id() == State::PRIMARY_STATE_UNCONFIGURED);
  }

  SECTION("Malformed water current")
  {
    auto sim = make_sim({{"water_current", std::vector<double>{0.1, 0.2}}});
    add_node(sim);
    REQUIRE(sim->trigger_transition(Transition::TRANSITION_CONFIGURE).id() == State::PRIMARY_STATE_UNCONFIGURED);
  }
}
