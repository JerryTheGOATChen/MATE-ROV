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

// Tests for your localizer and tracker.
//
// Test cases tagged [.hackathon] are hidden until you are ready for them. Run them with
//   ./build/ghost_brain/test_sensing "[hackathon]"
// and when your code passes, delete the "[.hackathon]" tag's dot so they run with colcon test too.

#include <cmath>
#include <cstring>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "ghost_brain/localizer.hpp"
#include "ghost_brain/tracker.hpp"

using Catch::Matchers::WithinAbs;
using ghost_brain::depth_at;
using ghost_brain::Localizer;
using ghost_brain::Tracker;

namespace
{

geometry_msgs::msg::Pose pose_at(double x, double y, double z, double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.z = std::sin(yaw / 2.0);
  pose.orientation.w = std::cos(yaw / 2.0);
  return pose;
}

/// A depth image like the house's camera: 128 x 72 pixels, everything at the same depth
sensor_msgs::msg::Image flat_depth(float depth)
{
  sensor_msgs::msg::Image image;
  image.width = 128;
  image.height = 72;
  image.encoding = "32FC1";
  image.step = image.width * sizeof(float);
  image.data.resize(image.step * image.height);
  for (size_t i = 0; i < image.width * image.height; ++i) {
    std::memcpy(&image.data[i * sizeof(float)], &depth, sizeof(float));
  }
  return image;
}

/// The house camera's intrinsics: 120 degrees across 128 pixels
sensor_msgs::msg::CameraInfo camera_info()
{
  sensor_msgs::msg::CameraInfo info;
  info.width = 128;
  info.height = 72;
  const double f = 64.0 / std::tan(M_PI / 3.0);
  info.k = {f, 0.0, 64.0, 0.0, f, 36.0, 0.0, 0.0, 1.0};
  return info;
}

}  // namespace

TEST_CASE("depth_at reads a depth image", "[sensing]")
{
  const auto image = flat_depth(2.5f);
  REQUIRE(depth_at(image, 0, 0) == 2.5f);
  REQUIRE(depth_at(image, 127, 71) == 2.5f);
  REQUIRE(std::isnan(depth_at(image, 128, 0)));
}

TEST_CASE("The localizer starts where it is told", "[sensing]")
{
  Localizer localizer;
  localizer.reset(pose_at(1.0, 2.0, 1.5, 0.3));
  REQUIRE(localizer.pose().position.x == 1.0);
  REQUIRE(localizer.pose().position.y == 2.0);
}

TEST_CASE("The localizer follows its velocity", "[.hackathon]")
{
  Localizer localizer;
  localizer.reset(pose_at(0.0, 0.0, 1.5, M_PI / 2.0));

  sensor_msgs::msg::Imu imu;
  imu.orientation = pose_at(0.0, 0.0, 0.0, M_PI / 2.0).orientation;
  localizer.on_imu(imu);

  // One second at 0.5 m/s forward while facing north, in 25 Hz steps
  for (int i = 0; i <= 25; ++i) {
    geometry_msgs::msg::TwistStamped velocity;
    velocity.header.frame_id = "base_link";
    velocity.header.stamp = rclcpp::Time(static_cast<int64_t>(i) * 40'000'000);
    velocity.twist.linear.x = 0.5;
    localizer.on_velocity(velocity);
  }

  REQUIRE_THAT(localizer.pose().position.x, WithinAbs(0.0, 0.05));
  REQUIRE_THAT(localizer.pose().position.y, WithinAbs(0.5, 0.05));
  REQUIRE_THAT(localizer.pose().position.z, WithinAbs(1.5, 0.05));
}

TEST_CASE("The tracker places what the camera sees", "[.hackathon]")
{
  Tracker tracker;

  // Candy dead ahead in the middle of the image, 2 m from the camera
  vision_msgs::msg::Detection2DArray detections;
  vision_msgs::msg::Detection2D candy;
  candy.bbox.center.position.x = 64.0;
  candy.bbox.center.position.y = 36.0;
  candy.bbox.size_x = 6.0;
  candy.bbox.size_y = 6.0;
  vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
  hypothesis.hypothesis.class_id = "candy";
  candy.results.push_back(hypothesis);
  detections.detections.push_back(candy);

  // The ghost is at (1, 2, 1.5) facing east, so the camera is at (1.25, 2, 1.6)
  tracker.on_detections(detections, flat_depth(2.0f), camera_info(), pose_at(1.0, 2.0, 1.5, 0.0));

  REQUIRE(tracker.targets().size() == 1);
  const auto & target = tracker.targets().front();
  REQUIRE_FALSE(target.is_person);
  REQUIRE_THAT(target.position.x, WithinAbs(3.25, 0.1));
  REQUIRE_THAT(target.position.y, WithinAbs(2.0, 0.1));
  REQUIRE_THAT(target.position.z, WithinAbs(1.6, 0.1));
}
