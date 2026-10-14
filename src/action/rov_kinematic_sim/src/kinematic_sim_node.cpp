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

#include "rov_kinematic_sim/kinematic_sim_node.hpp"

#include <algorithm>
#include <chrono>
#include <Eigen/Geometry>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace rov_kinematic_sim
{
namespace
{

Eigen::Vector3d to_vector3(const std::vector<double> & values, const std::string & name)
{
  if (values.size() != 3) {
    throw std::invalid_argument(name + " must have three elements: x, y, z");
  }
  return Eigen::Vector3d(values[0], values[1], values[2]);
}

}  // namespace

KinematicSimNode::KinematicSimNode(const rclcpp::NodeOptions & options)
: LifecycleNode("kinematic_sim", options)
{
  declare_parameter<double>("update_rate_hz", 50.0);
  declare_parameter<double>("cmd_vel_timeout_s", 0.5);
  declare_parameter<std::string>("odom_frame", "odom");
  declare_parameter<std::string>("base_frame", "base_link");
  declare_parameter<std::vector<double>>("initial_pose", {0.0, 0.0, 0.0, 0.0});
  declare_parameter<double>("response_time_s", 0.3);
  declare_parameter<std::vector<double>>("max_linear_speed", {1.0, 0.6, 0.6});
  declare_parameter<double>("max_yaw_rate", 1.0);
  declare_parameter<std::vector<double>>("water_current", {0.0, 0.0, 0.0});
  declare_parameter<double>("surface_z", 0.0);
}

KinematicSimNode::CallbackReturn KinematicSimNode::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    load_parameters();
  } catch (const std::invalid_argument & e) {
    RCLCPP_ERROR(get_logger(), "Invalid parameters: %s", e.what());
    return CallbackReturn::FAILURE;
  }

  state_ = initial_state_;
  command_ = BodyVelocity{};
  command_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "cmd_vel", 10, [this](const geometry_msgs::msg::TwistStamped & msg) { on_cmd_vel(msg); });
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  RCLCPP_INFO(
    get_logger(),
    "Configured: %.0f Hz, response time %.2f s, water current [%.2f %.2f %.2f] m/s",
    update_rate_hz_,
    model_.response_time,
    model_.water_current.x(),
    model_.water_current.y(),
    model_.water_current.z());
  return CallbackReturn::SUCCESS;
}

KinematicSimNode::CallbackReturn KinematicSimNode::on_activate(const rclcpp_lifecycle::State &)
{
  odom_pub_->on_activate();
  const auto period =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / update_rate_hz_));
  timer_ = create_wall_timer(period, [this]() { update(); });

  RCLCPP_INFO(get_logger(), "Activated");
  return CallbackReturn::SUCCESS;
}

KinematicSimNode::CallbackReturn KinematicSimNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  odom_pub_->on_deactivate();

  RCLCPP_INFO(get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

KinematicSimNode::CallbackReturn KinematicSimNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  release_resources();
  RCLCPP_INFO(get_logger(), "Cleaned up");
  return CallbackReturn::SUCCESS;
}

KinematicSimNode::CallbackReturn KinematicSimNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  release_resources();
  RCLCPP_INFO(get_logger(), "Shut down");
  return CallbackReturn::SUCCESS;
}

void KinematicSimNode::load_parameters()
{
  update_rate_hz_ = get_parameter("update_rate_hz").as_double();
  const double cmd_vel_timeout_s = get_parameter("cmd_vel_timeout_s").as_double();
  odom_frame_ = get_parameter("odom_frame").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  const auto initial_pose = get_parameter("initial_pose").as_double_array();

  model_.response_time = get_parameter("response_time_s").as_double();
  model_.max_linear_speed = to_vector3(get_parameter("max_linear_speed").as_double_array(), "max_linear_speed");
  model_.max_yaw_rate = get_parameter("max_yaw_rate").as_double();
  model_.water_current = to_vector3(get_parameter("water_current").as_double_array(), "water_current");
  model_.surface_z = get_parameter("surface_z").as_double();

  if (update_rate_hz_ <= 0.0 || cmd_vel_timeout_s <= 0.0) {
    throw std::invalid_argument("update_rate_hz and cmd_vel_timeout_s must be positive");
  }
  if (model_.response_time < 0.0 || model_.max_yaw_rate <= 0.0 || (model_.max_linear_speed.array() <= 0.0).any()) {
    throw std::invalid_argument("response_time_s must be non-negative and the speed limits positive");
  }
  if (initial_pose.size() != 4) {
    throw std::invalid_argument("initial_pose must be [x, y, z, yaw]");
  }
  if (initial_pose[2] > model_.surface_z) {
    throw std::invalid_argument("initial_pose must not be above surface_z");
  }

  cmd_vel_timeout_ = rclcpp::Duration::from_seconds(cmd_vel_timeout_s);
  initial_state_ = VehicleState{};
  initial_state_.position = Eigen::Vector3d(initial_pose[0], initial_pose[1], initial_pose[2]);
  initial_state_.yaw = initial_pose[3];
}

void KinematicSimNode::on_cmd_vel(const geometry_msgs::msg::TwistStamped & msg)
{
  if (!msg.header.frame_id.empty() && msg.header.frame_id != base_frame_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Ignoring cmd_vel in frame '%s': commands must be in '%s'",
      msg.header.frame_id.c_str(),
      base_frame_.c_str());
    return;
  }

  command_.linear = Eigen::Vector3d(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z);
  command_.yaw_rate = msg.twist.angular.z;
  command_time_ = now();
}

void KinematicSimNode::update()
{
  const rclcpp::Time stamp = now();
  const double dt = 1.0 / update_rate_hz_;

  // If the commander goes quiet, the vehicle stops thrusting
  const BodyVelocity command = stamp - command_time_ <= cmd_vel_timeout_ ? command_ : BodyVelocity{};

  const VehicleState previous = state_;
  state_ = step(state_, command, model_, dt);

  // Report velocity over ground, which includes the current and stopping at the surface
  const Eigen::Vector3d world_velocity = (state_.position - previous.position) / dt;
  publish_state(stamp, Eigen::AngleAxisd(-state_.yaw, Eigen::Vector3d::UnitZ()) * world_velocity);
}

void KinematicSimNode::publish_state(const rclcpp::Time & stamp, const Eigen::Vector3d & body_velocity)
{
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, state_.yaw);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose.position.x = state_.position.x();
  odom.pose.pose.position.y = state_.position.y();
  odom.pose.pose.position.z = state_.position.z();
  odom.pose.pose.orientation = tf2::toMsg(orientation);
  odom.twist.twist.linear.x = body_velocity.x();
  odom.twist.twist.linear.y = body_velocity.y();
  odom.twist.twist.linear.z = body_velocity.z();
  odom.twist.twist.angular.z = state_.velocity.yaw_rate;
  odom_pub_->publish(odom);

  geometry_msgs::msg::TransformStamped transform;
  transform.header = odom.header;
  transform.child_frame_id = base_frame_;
  transform.transform.translation.x = state_.position.x();
  transform.transform.translation.y = state_.position.y();
  transform.transform.translation.z = state_.position.z();
  transform.transform.rotation = odom.pose.pose.orientation;
  tf_broadcaster_->sendTransform(transform);
}

void KinematicSimNode::release_resources()
{
  timer_.reset();
  cmd_vel_sub_.reset();
  odom_pub_.reset();
  tf_broadcaster_.reset();
}

}  // namespace rov_kinematic_sim

RCLCPP_COMPONENTS_REGISTER_NODE(rov_kinematic_sim::KinematicSimNode)
