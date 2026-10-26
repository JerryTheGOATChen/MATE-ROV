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
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include "ghost_brain/ghost_brain_node.hpp"
#include "test_fixtures/test_executor_fixture.hpp"
#include "test_nodes/subscriber_test_node.hpp"

using ghost_brain::GhostBrainNode;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;
using wato::test::SubscriberTestNode;
using wato::test::TestExecutorFixture;

using TwistStamped = geometry_msgs::msg::TwistStamped;

namespace
{

// A namespace of its own, so the test cannot interfere with a running haunted house
constexpr char kNamespace[] = "/ghost_brain_test";
constexpr char kCmdVelTopic[] = "/ghost_brain_test/cmd_vel";

std::shared_ptr<GhostBrainNode> make_brain(std::vector<rclcpp::Parameter> overrides = {})
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(overrides);
  options.arguments({"--ros-args", "-r", std::string("__ns:=") + kNamespace});
  return std::make_shared<GhostBrainNode>(options);
}

}  // namespace

TEST_CASE_METHOD(TestExecutorFixture, "GhostBrainNode holds still until the game starts", "[ghost_brain]")
{
  auto brain = make_brain();
  auto cmd_sub = std::make_shared<SubscriberTestNode<TwistStamped>>(kCmdVelTopic, "cmd_sub");
  add_node(brain);
  add_node(cmd_sub);
  start_spinning();

  REQUIRE(brain->trigger_transition(Transition::TRANSITION_CONFIGURE).id() == State::PRIMARY_STATE_INACTIVE);
  REQUIRE(brain->trigger_transition(Transition::TRANSITION_ACTIVATE).id() == State::PRIMARY_STATE_ACTIVE);
  REQUIRE(cmd_sub->wait_for_publishers(1));

  for (int i = 0; i < 5; ++i) {
    auto future = cmd_sub->expect_next_message();
    REQUIRE(future.wait_for(std::chrono::seconds(2)) == std::future_status::ready);
    const TwistStamped cmd = future.get();
    REQUIRE(cmd.header.frame_id == "base_link");
    REQUIRE(cmd.twist.linear.x == 0.0);
    REQUIRE(cmd.twist.angular.z == 0.0);
  }
}

TEST_CASE_METHOD(TestExecutorFixture, "GhostBrainNode rejects a non-positive control rate", "[ghost_brain]")
{
  auto brain = make_brain({{"control_rate_hz", 0.0}});
  add_node(brain);
  REQUIRE(brain->trigger_transition(Transition::TRANSITION_CONFIGURE).id() == State::PRIMARY_STATE_UNCONFIGURED);
}
