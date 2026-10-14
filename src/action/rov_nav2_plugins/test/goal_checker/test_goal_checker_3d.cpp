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

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rov_nav2_plugins/common/pose4d.hpp"
#include "rov_nav2_plugins/goal_checker/goal_checker_3d.hpp"
#include "test_fixtures/test_executor_fixture.hpp"

using Catch::Matchers::WithinAbs;
using rov_nav2_plugins::GoalChecker3D;
using wato::test::TestExecutorFixture;

namespace
{

const char * const kPluginName = "goal_checker";

GoalChecker3D initialized_checker(std::vector<rclcpp::Parameter> overrides)
{
  for (auto & override : overrides) {
    override = rclcpp::Parameter(std::string(kPluginName) + "." + override.get_name(), override.get_parameter_value());
  }
  rclcpp::NodeOptions options;
  options.parameter_overrides(overrides);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("goal_checker_test_node", options);

  GoalChecker3D checker;
  checker.initialize(node, kPluginName, nullptr);
  return checker;
}

geometry_msgs::msg::Pose pose(double x, double y, double z, double yaw)
{
  return rov_nav2_plugins::to_pose_msg({{x, y, z}, yaw});
}

const geometry_msgs::msg::Pose kGoal = pose(1.0, 2.0, -3.0, 0.5);
const geometry_msgs::msg::Twist kStill;

}  // namespace

TEST_CASE_METHOD(TestExecutorFixture, "GoalChecker3D needs position and heading within tolerance", "[goal_checker]")
{
  auto checker =
    initialized_checker({{"xy_goal_tolerance", 0.2}, {"z_goal_tolerance", 0.1}, {"yaw_goal_tolerance", 0.1}});

  SECTION("At the goal")
  {
    REQUIRE(checker.isGoalReached(kGoal, kGoal, kStill));
  }

  SECTION("Within every tolerance")
  {
    REQUIRE(checker.isGoalReached(pose(1.1, 2.1, -3.05, 0.55), kGoal, kStill));
  }

  SECTION("Too far horizontally")
  {
    REQUIRE_FALSE(checker.isGoalReached(pose(1.2, 2.1, -3.0, 0.5), kGoal, kStill));
  }

  SECTION("Too far vertically, which Nav2's 2D goal checkers would accept")
  {
    REQUIRE_FALSE(checker.isGoalReached(pose(1.0, 2.0, -3.2, 0.5), kGoal, kStill));
  }

  SECTION("Wrong heading")
  {
    REQUIRE_FALSE(checker.isGoalReached(pose(1.0, 2.0, -3.0, 0.7), kGoal, kStill));
  }
}

TEST_CASE_METHOD(TestExecutorFixture, "GoalChecker3D keeps checking position while turning", "[goal_checker]")
{
  auto checker = initialized_checker({});

  REQUIRE_FALSE(checker.isGoalReached(pose(1.0, 2.0, -3.0, 1.5), kGoal, kStill));
  // Drifted off position while turning to the goal heading
  REQUIRE_FALSE(checker.isGoalReached(pose(1.5, 2.0, -3.0, 0.5), kGoal, kStill));
}

TEST_CASE_METHOD(TestExecutorFixture, "GoalChecker3D reports its tolerances", "[goal_checker]")
{
  auto checker =
    initialized_checker({{"xy_goal_tolerance", 0.2}, {"z_goal_tolerance", 0.1}, {"yaw_goal_tolerance", 0.3}});

  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist velocity_tolerance;
  REQUIRE(checker.getTolerances(pose_tolerance, velocity_tolerance));

  REQUIRE(pose_tolerance.position.x == 0.2);
  REQUIRE(pose_tolerance.position.y == 0.2);
  REQUIRE(pose_tolerance.position.z == 0.1);
  REQUIRE_THAT(rov_nav2_plugins::to_pose4d(pose_tolerance).yaw, WithinAbs(0.3, 1e-9));
  // Velocity is not checked
  REQUIRE(velocity_tolerance.linear.x == std::numeric_limits<double>::lowest());
  REQUIRE(velocity_tolerance.angular.z == std::numeric_limits<double>::lowest());
}

TEST_CASE_METHOD(TestExecutorFixture, "GoalChecker3D rejects non-positive tolerances", "[goal_checker]")
{
  REQUIRE_THROWS_AS(initialized_checker({{"z_goal_tolerance", 0.0}}), std::invalid_argument);
}
