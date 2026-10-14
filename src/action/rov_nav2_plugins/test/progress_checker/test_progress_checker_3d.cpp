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

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "rcl/time.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rov_nav2_plugins/common/pose4d.hpp"
#include "rov_nav2_plugins/progress_checker/progress_checker_3d.hpp"
#include "test_fixtures/test_executor_fixture.hpp"

using rov_nav2_plugins::ProgressChecker3D;
using wato::test::TestExecutorFixture;

namespace
{

const char * const kPluginName = "progress_checker";

/// Node whose ROS clock the test sets directly, so time allowances are checked without sleeping.
class ManualClockNode
{
public:
  explicit ManualClockNode(std::vector<rclcpp::Parameter> overrides)
  {
    for (auto & override : overrides) {
      override =
        rclcpp::Parameter(std::string(kPluginName) + "." + override.get_name(), override.get_parameter_value());
    }
    // With simulated time and no /clock publisher, the clock only moves when set_time() is called
    overrides.emplace_back("use_sim_time", true);
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("progress_checker_test_node", options);
    set_time(1.0);
  }

  void set_time(double seconds)
  {
    const auto nanoseconds = static_cast<rcl_time_point_value_t>(seconds * 1e9);
    if (rcl_set_ros_time_override(node_->get_clock()->get_clock_handle(), nanoseconds) != RCL_RET_OK) {
      throw std::runtime_error("Failed to set the node's ROS time");
    }
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node() const
  {
    return node_;
  }

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};

geometry_msgs::msg::PoseStamped pose(double x, double y, double z, double yaw)
{
  geometry_msgs::msg::PoseStamped stamped;
  stamped.pose = rov_nav2_plugins::to_pose_msg({{x, y, z}, yaw});
  return stamped;
}

}  // namespace

TEST_CASE_METHOD(
  TestExecutorFixture, "ProgressChecker3D tracks movement within the time allowance", "[progress_checker]")
{
  ManualClockNode clock(
    {{"required_movement_radius", 0.5}, {"required_movement_angle", 0.5}, {"movement_time_allowance", 10.0}});
  ProgressChecker3D checker;
  checker.initialize(clock.node(), kPluginName);

  auto start = pose(0.0, 0.0, 0.0, 0.0);
  REQUIRE(checker.check(start));

  SECTION("Diving counts as progress, unlike Nav2's 2D progress checkers")
  {
    clock.set_time(20.0);
    auto dived = pose(0.0, 0.0, -0.6, 0.0);
    REQUIRE(checker.check(dived));
  }

  SECTION("Turning in place counts as progress")
  {
    clock.set_time(20.0);
    auto turned = pose(0.0, 0.0, 0.0, 0.6);
    REQUIRE(checker.check(turned));
  }

  SECTION("Small movements are allowed until the time allowance runs out")
  {
    auto nudged = pose(0.2, 0.0, -0.2, 0.2);
    clock.set_time(10.0);
    REQUIRE(checker.check(nudged));
    clock.set_time(11.5);
    REQUIRE_FALSE(checker.check(nudged));

    // A new navigation task starts the allowance over
    checker.reset();
    REQUIRE(checker.check(nudged));
  }
}

TEST_CASE_METHOD(TestExecutorFixture, "ProgressChecker3D rejects non-positive parameters", "[progress_checker]")
{
  ManualClockNode clock({{"movement_time_allowance", 0.0}});
  ProgressChecker3D checker;
  REQUIRE_THROWS_AS(checker.initialize(clock.node(), kPluginName), std::invalid_argument);
}
