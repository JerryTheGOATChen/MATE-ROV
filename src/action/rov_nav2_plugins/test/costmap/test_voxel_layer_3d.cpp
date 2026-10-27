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

#include <atomic>
#include <chrono>
#include <Eigen/Core>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rov_nav2_plugins/costmap/voxel_layer_3d.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "test_fixtures/test_executor_fixture.hpp"
#include "tf2_ros/buffer.hpp"

using rov_nav2_plugins::VoxelLayer3D;
using wato::test::TestExecutorFixture;

namespace
{

const char * const kTopic = "/test_points";
const Eigen::Vector3d kSonar{0.5, 1.5, -0.5};  // the sonar's position in "map"

geometry_msgs::msg::Point corner(double x, double y)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  return point;
}

/// A cloud in the sonar's frame of the given points, which are in "map".
sensor_msgs::msg::PointCloud2 cloud_of(const std::vector<Eigen::Vector3d> & points, const rclcpp::Time & stamp)
{
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

/// A floor 0.95 m down across the whole 4 x 3 m costmap, and a post through every depth at (2.05, 1.55).
std::vector<Eigen::Vector3d> floor_and_post()
{
  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < 40; ++i) {
    for (int j = 0; j < 30; ++j) {
      points.emplace_back(0.05 + 0.1 * i, 0.05 + 0.1 * j, -0.95);
    }
  }
  for (int k = 0; k < 10; ++k) {
    points.emplace_back(2.05, 1.55, -0.95 + 0.1 * k);
  }
  return points;
}

/// The layer in a 4 x 3 m costmap of 0.1 m cells with its origin at (0, 0) in "map", for a vehicle 0.6 m
/// wide and 0.3 m tall, with the sonar fixed in "map".
struct Setup
{
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
  std::unique_ptr<tf2_ros::Buffer> tf;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> costmap;
  std::shared_ptr<VoxelLayer3D> layer;

  Setup()
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      {"voxel_layer.topic", kTopic},
      {"voxel_layer.min_z", -1.0},
      {"voxel_layer.max_z", 0.0},
      {"voxel_layer.robot_half_height", 0.15},
      {"voxel_layer.inflation_radius", 0.6},
      {"voxel_layer.publish_voxel_costs", true},
    });
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("voxel_layer_test_node", options);

    tf = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    geometry_msgs::msg::TransformStamped sonar;
    sonar.header.frame_id = "map";
    sonar.child_frame_id = "sonar";
    sonar.transform.translation.x = kSonar.x();
    sonar.transform.translation.y = kSonar.y();
    sonar.transform.translation.z = kSonar.z();
    sonar.transform.rotation.w = 1.0;
    tf->setTransform(sonar, "test", true);

    costmap = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    costmap->resizeMap(40, 30, 0.1, 0.0, 0.0);
    costmap->setFootprint({corner(0.3, 0.3), corner(0.3, -0.3), corner(-0.3, -0.3), corner(-0.3, 0.3)});

    layer = std::make_shared<VoxelLayer3D>();
    costmap->addPlugin(layer);
    layer->initialize(costmap.get(), "voxel_layer", tf.get(), node, nullptr);
    layer->activate();
  }

  unsigned char cost_2d(unsigned int mx, unsigned int my) const
  {
    return costmap->getCostmap()->getCost(mx, my);
  }
};

}  // namespace

TEST_CASE_METHOD(TestExecutorFixture, "VoxelLayer3D keeps the floor in 3D and only full columns in 2D", "[voxel_layer]")
{
  Setup setup;
  auto publisher_node = std::make_shared<rclcpp::Node>("cloud_publisher");
  auto publisher = publisher_node->create_publisher<sensor_msgs::msg::PointCloud2>(kTopic, 10);
  std::atomic<std::size_t> voxel_costs_points{0};
  auto voxel_costs_sub = publisher_node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/voxel_layer/voxel_costs", 10, [&voxel_costs_points](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
      voxel_costs_points = static_cast<std::size_t>(cloud->width) * cloud->height;
    });
  add_node(setup.node);
  add_node(publisher_node);
  start_spinning();

  // Publish until the layer has the cloud, waiting out discovery
  const auto floor_voxel = [&setup] {
    const auto volume = setup.layer->cost_volume();
    return volume && volume->cost({5, 5, 0}) == nav2_costmap_2d::LETHAL_OBSTACLE;
  };
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (!floor_voxel() && std::chrono::steady_clock::now() < deadline) {
    publisher->publish(cloud_of(floor_and_post(), setup.node->now()));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    setup.costmap->updateMap(0.0, 0.0, 0.0);
  }
  REQUIRE(floor_voxel());

  const auto volume = setup.layer->cost_volume();
  // The floor is lethal in 3D, and the water well above it is free
  REQUIRE(volume->cost({5, 5, 9}) == nav2_costmap_2d::FREE_SPACE);
  // In 2D, only the post and the cells within the vehicle's half-width of it are blocked
  REQUIRE(setup.cost_2d(20, 15) == nav2_costmap_2d::LETHAL_OBSTACLE);
  REQUIRE(setup.cost_2d(22, 15) == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  REQUIRE(setup.cost_2d(5, 5) == nav2_costmap_2d::FREE_SPACE);

  // The inflated voxels are published for viewing
  const auto published_by = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (voxel_costs_points == 0 && std::chrono::steady_clock::now() < published_by) {
    publisher->publish(cloud_of(floor_and_post(), setup.node->now()));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    setup.costmap->updateMap(0.0, 0.0, 0.0);
  }
  REQUIRE(voxel_costs_points > 1200);

  SECTION("Clearing the costmap forgets every voxel")
  {
    // Let the clouds still in flight arrive first, so none marks the grid again
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    setup.layer->reset();
    setup.costmap->updateMap(0.0, 0.0, 0.0);
    REQUIRE(setup.layer->cost_volume()->cost({5, 5, 0}) == nav2_costmap_2d::FREE_SPACE);
  }
}
