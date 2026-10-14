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
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rov_nav2_plugins/common/pose4d.hpp"
#include "rov_nav2_plugins/controller/pure_pursuit_controller_3d.hpp"
#include "test_fixtures/test_executor_fixture.hpp"

using Catch::Matchers::WithinAbs;
using rov_nav2_plugins::PurePursuitController3D;
using wato::test::TestExecutorFixture;

namespace
{

const char * const kPluginName = "FollowPath";
constexpr double kTolerance = 1e-9;

// Largest sway change in one 20 Hz control period with the default max_accel_y
constexpr double kSwayStep = 0.3 * 0.05;

/// The odom frame sits 1 m to the left (+y) of map, so the odom origin is 1 m off a path along map's x axis.
std::shared_ptr<tf2_ros::Buffer> tf_buffer_with_odom_offset(const rclcpp::Clock::SharedPtr & clock)
{
  auto buffer = std::make_shared<tf2_ros::Buffer>(clock);
  // Transforms are inserted directly, so lookups never wait for a listener thread
  buffer->setUsingDedicatedThread(true);

  geometry_msgs::msg::TransformStamped map_to_odom;
  map_to_odom.header.frame_id = "map";
  map_to_odom.child_frame_id = "odom";
  map_to_odom.transform.translation.y = 1.0;
  map_to_odom.transform.rotation.w = 1.0;
  buffer->setTransform(map_to_odom, "test", true);
  return buffer;
}

/// Controller configured as the controller server would, on a node with the given parameter overrides.
std::unique_ptr<PurePursuitController3D> configured_controller(const std::vector<rclcpp::Parameter> & overrides)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(overrides);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_test_node", options);

  // The controller stamps commands with the costmap's robot base frame, "base_link" by default
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_local_costmap");
  costmap->on_configure(rclcpp_lifecycle::State());

  auto controller = std::make_unique<PurePursuitController3D>();
  controller->configure(node, kPluginName, tf_buffer_with_odom_offset(node->get_clock()), costmap);
  return controller;
}

/// Path along map's x axis from the origin to (`length`, 0, 0).
nav_msgs::msg::Path path_along_map_x(double length)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (const rov_nav2_plugins::Pose4D & pose :
       {rov_nav2_plugins::Pose4D{{0.0, 0.0, 0.0}, 0.0}, rov_nav2_plugins::Pose4D{{length, 0.0, 0.0}, 0.0}})
  {
    geometry_msgs::msg::PoseStamped stamped;
    stamped.header.frame_id = "map";
    stamped.pose = rov_nav2_plugins::to_pose_msg(pose);
    path.poses.push_back(stamped);
  }
  return path;
}

geometry_msgs::msg::PoseStamped robot_at_odom_origin()
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "odom";
  pose.pose.orientation.w = 1.0;
  return pose;
}

const geometry_msgs::msg::Twist kUnusedVelocity;

}  // namespace

TEST_CASE_METHOD(TestExecutorFixture, "PurePursuitController3D steers toward the path across frames", "[controller]")
{
  auto controller = configured_controller({});
  controller->setPlan(path_along_map_x(5.0));

  const auto cmd_vel = controller->computeVelocityCommands(robot_at_odom_origin(), kUnusedVelocity, nullptr);

  REQUIRE(cmd_vel.header.frame_id == "base_link");
  // The robot is 1 m left of the path in map, so it heads for the lookahead point 0.6 m ahead on the
  // path: direction (0.6, -1), accelerating from rest until sway reaches its limit
  REQUIRE_THAT(cmd_vel.twist.linear.y, WithinAbs(-kSwayStep, kTolerance));
  REQUIRE_THAT(cmd_vel.twist.linear.x, WithinAbs(0.6 * kSwayStep, kTolerance));
  REQUIRE_THAT(cmd_vel.twist.linear.z, WithinAbs(0.0, kTolerance));
}

TEST_CASE_METHOD(TestExecutorFixture, "PurePursuitController3D needs a non-empty path", "[controller]")
{
  auto controller = configured_controller({});

  REQUIRE_THROWS_AS(controller->setPlan(nav_msgs::msg::Path()), nav2_core::InvalidPath);
  REQUIRE_THROWS_AS(
    controller->computeVelocityCommands(robot_at_odom_origin(), kUnusedVelocity, nullptr), nav2_core::InvalidPath);

  SECTION("reset() drops the path at the end of a task")
  {
    controller->setPlan(path_along_map_x(5.0));
    controller->reset();
    REQUIRE_THROWS_AS(
      controller->computeVelocityCommands(robot_at_odom_origin(), kUnusedVelocity, nullptr), nav2_core::InvalidPath);
  }
}

TEST_CASE_METHOD(TestExecutorFixture, "PurePursuitController3D reports an unknown robot frame", "[controller]")
{
  auto controller = configured_controller({{std::string(kPluginName) + ".transform_tolerance", 0.0}});
  controller->setPlan(path_along_map_x(5.0));

  auto pose = robot_at_odom_origin();
  pose.header.frame_id = "unknown_frame";
  REQUIRE_THROWS_AS(controller->computeVelocityCommands(pose, kUnusedVelocity, nullptr), nav2_core::ControllerTFError);
}

TEST_CASE_METHOD(TestExecutorFixture, "PurePursuitController3D rejects invalid limits", "[controller]")
{
  REQUIRE_THROWS_AS(configured_controller({{std::string(kPluginName) + ".max_vel_z", 0.0}}), std::invalid_argument);
}
