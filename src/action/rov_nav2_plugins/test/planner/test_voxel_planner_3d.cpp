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

#include <algorithm>
#include <chrono>
#include <Eigen/Core>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rov_nav2_plugins/costmap/voxel_layer_3d.hpp"
#include "rov_nav2_plugins/planner/voxel_planner_3d.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "test_fixtures/test_executor_fixture.hpp"

using Catch::Matchers::WithinAbs;
using rov_nav2_plugins::VoxelLayer3D;
using rov_nav2_plugins::VoxelPlanner3D;
using wato::test::TestExecutorFixture;

namespace
{

const char * const kTopic = "/test_planner_points";
const Eigen::Vector3d kSonar{0.5, 1.5, -1.0};  // the sonar's position in "map"

geometry_msgs::msg::PoseStamped map_pose(double x, double y, double z)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;
  pose.pose.orientation.w = 1.0;
  return pose;
}

/// What the sonar sees of a 6 x 3 x 3 m tank: the floor, whose top is at z = -2.9, and a block from wall to
/// wall from x = 2.5 to 3.5, 0.8 m tall. Returns are at voxel centers, in the sonar's frame.
sensor_msgs::msg::PointCloud2 tank_cloud(const rclcpp::Time & stamp)
{
  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < 60; ++i) {
    for (int j = 0; j < 30; ++j) {
      const double x = 0.05 + 0.1 * i;
      const double y = 0.05 + 0.1 * j;
      points.emplace_back(x, y, -2.95);
      if (x > 2.5 && x < 3.5) {
        for (int k = 1; k < 9; ++k) {
          points.emplace_back(x, y, -2.95 + 0.1 * k);
        }
      }
    }
  }

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "sonar";
  cloud.header.stamp = stamp;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  for (const auto & point : points) {
    const Eigen::Vector3d in_sonar = point - kSonar;
    *x = static_cast<float>(in_sonar.x());
    *y = static_cast<float>(in_sonar.y());
    *z = static_cast<float>(in_sonar.z());
    ++x;
    ++y;
    ++z;
  }
  return cloud;
}

/// The planner on a global costmap that loads VoxelLayer3D by name, configured as the planner server would:
/// the costmap first, then the planner. The costmap is 6 x 3 m with its origin at (0, 0) in "map".
struct Setup
{
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
  std::unique_ptr<VoxelPlanner3D> planner;

  explicit Setup(const std::vector<rclcpp::Parameter> & planner_parameters = {})
  {
    rclcpp::NodeOptions costmap_options;
    costmap_options.parameter_overrides({
      {"global_frame", "map"},
      {"robot_base_frame", "base_link"},
      {"rolling_window", false},
      {"width", 6},
      {"height", 3},
      {"resolution", 0.1},
      {"origin_x", 0.0},
      {"origin_y", 0.0},
      {"robot_radius", 0.35},
      {"plugins", std::vector<std::string>{"voxel_layer_3d"}},
      {"voxel_layer_3d.plugin", "rov_nav2_plugins::VoxelLayer3D"},
      {"voxel_layer_3d.topic", kTopic},
      {"voxel_layer_3d.min_z", -3.0},
      {"voxel_layer_3d.max_z", 0.0},
      {"voxel_layer_3d.robot_half_height", 0.2},
      {"voxel_layer_3d.inflation_radius", 1.0},
    });
    costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(costmap_options);
    costmap->on_configure(rclcpp_lifecycle::State());

    geometry_msgs::msg::TransformStamped sonar;
    sonar.header.frame_id = "map";
    sonar.child_frame_id = "sonar";
    sonar.transform.translation.x = kSonar.x();
    sonar.transform.translation.y = kSonar.y();
    sonar.transform.translation.z = kSonar.z();
    sonar.transform.rotation.w = 1.0;
    costmap->getTfBuffer()->setTransform(sonar, "test", true);

    rclcpp::NodeOptions node_options;
    node_options.parameter_overrides(planner_parameters);
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("voxel_planner_test_node", node_options);
    planner = std::make_unique<VoxelPlanner3D>();
  }

  void configure()
  {
    planner->configure(node, "VoxelPlanner3D", nullptr, costmap);
    planner->activate();
  }

  std::shared_ptr<VoxelLayer3D> layer() const
  {
    return std::dynamic_pointer_cast<VoxelLayer3D>(costmap->getLayeredCostmap()->getPlugins()->front());
  }
};

bool never_canceled()
{
  return false;
}

}  // namespace

TEST_CASE_METHOD(TestExecutorFixture, "VoxelPlanner3D plans over an obstacle the sonar sees", "[voxel_planner]")
{
  Setup setup;
  setup.configure();
  auto publisher_node = std::make_shared<rclcpp::Node>("planner_cloud_publisher");
  auto publisher = publisher_node->create_publisher<sensor_msgs::msg::PointCloud2>(kTopic, 10);
  add_node(publisher_node);
  start_spinning();

  // Before the sonar has seen anything, all the water is unknown and free, so the route is straight
  const auto blind = setup.planner->createPlan(map_pose(0.5, 1.5, -2.5), map_pose(5.5, 1.5, -2.5), never_canceled);
  REQUIRE(blind.poses.size() == 2);

  // Publish until the layer has the block, waiting out discovery
  const auto has_block = [&setup] {
    const auto volume = setup.layer()->cost_volume();
    return volume && volume->cost({30, 15, 5}) == nav2_costmap_2d::LETHAL_OBSTACLE;
  };
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (!has_block() && std::chrono::steady_clock::now() < deadline) {
    publisher->publish(tank_cloud(setup.costmap->now()));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    setup.costmap->getLayeredCostmap()->updateMap(0.0, 0.0, 0.0);
  }
  REQUIRE(has_block());

  const auto path = setup.planner->createPlan(map_pose(0.5, 1.5, -2.5), map_pose(5.5, 1.5, -2.5), never_canceled);

  REQUIRE(path.header.frame_id == "map");
  REQUIRE(path.poses.size() > 2);
  REQUIRE_THAT(path.poses.front().pose.position.x, WithinAbs(0.5, 1e-9));
  REQUIRE_THAT(path.poses.back().pose.position.x, WithinAbs(5.5, 1e-9));
  REQUIRE_THAT(path.poses.back().pose.position.z, WithinAbs(-2.5, 1e-9));
  // The block spans the tank, so the route goes over it, keeping the vehicle's half-height above it
  double over_block = -10.0;
  for (const auto & pose : path.poses) {
    REQUIRE(pose.header.frame_id == "map");
    if (pose.pose.position.x > 2.5 && pose.pose.position.x < 3.5) {
      over_block = std::max(over_block, pose.pose.position.z);
    }
  }
  REQUIRE(over_block > -2.1 + 0.2 - 0.1);

  SECTION("Goals it cannot reach fail with Nav2's planner exceptions")
  {
    REQUIRE_THROWS_AS(
      setup.planner->createPlan(map_pose(0.5, 1.5, -2.5), map_pose(3.0, 1.5, -2.5), never_canceled),
      nav2_core::GoalOccupied);
    REQUIRE_THROWS_AS(
      setup.planner->createPlan(map_pose(0.5, 1.5, -2.5), map_pose(7.0, 1.5, -2.5), never_canceled),
      nav2_core::GoalOutsideMapBounds);
    REQUIRE_THROWS_AS(
      setup.planner->createPlan(map_pose(0.5, 1.5, -2.5), map_pose(5.5, 1.5, -2.5), [] { return true; }),
      nav2_core::PlannerCancelled);
  }
}

TEST_CASE_METHOD(TestExecutorFixture, "VoxelPlanner3D needs a VoxelLayer3D and valid parameters", "[voxel_planner]")
{
  SECTION("No layer by that name in the costmap")
  {
    Setup setup({{"VoxelPlanner3D.voxel_layer", "missing"}});
    REQUIRE_THROWS_AS(setup.configure(), std::runtime_error);
  }

  SECTION("An unknown heading mode")
  {
    Setup setup({{"VoxelPlanner3D.yaw_mode", "sideways"}});
    REQUIRE_THROWS_AS(setup.configure(), std::invalid_argument);
  }

  SECTION("An unknown cost the vehicle could not be in")
  {
    Setup setup({{"VoxelPlanner3D.unknown_cost", 253}});
    REQUIRE_THROWS_AS(setup.configure(), std::invalid_argument);
  }
}
