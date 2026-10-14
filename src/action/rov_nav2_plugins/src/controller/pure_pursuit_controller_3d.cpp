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

#include "rov_nav2_plugins/controller/pure_pursuit_controller_3d.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "nav2_core/controller_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rov_nav2_plugins/common/pose4d.hpp"

namespace rov_nav2_plugins
{

void PurePursuitController3D::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("PurePursuitController3D: unable to lock the parent node");
  }

  logger_ = node->get_logger();
  tf_ = std::move(tf);
  base_frame_ = costmap_ros->getBaseFrameID();

  const auto param = [&node, &name](const std::string & param_name, double default_value) {
    return nav2_util::declare_or_get_parameter(node, name + "." + param_name, default_value);
  };

  PursuitConfig config;
  config.lookahead_dist = param("lookahead_dist", 0.6);
  config.approach_gain = param("approach_gain", 1.5);
  config.max_vel.linear = Eigen::Vector3d(param("max_vel_x", 0.5), param("max_vel_y", 0.3), param("max_vel_z", 0.3));
  config.max_vel.yaw = param("max_vel_yaw", 0.6);
  config.max_accel.linear =
    Eigen::Vector3d(param("max_accel_x", 0.4), param("max_accel_y", 0.3), param("max_accel_z", 0.3));
  config.max_accel.yaw = param("max_accel_yaw", 0.6);
  transform_tolerance_ = param("transform_tolerance", 0.1);

  // Acceleration is limited per control period. The controller server owns this parameter.
  const double controller_frequency = nav2_util::declare_or_get_parameter(node, "controller_frequency", 20.0);
  if (controller_frequency <= 0.0 || transform_tolerance_ < 0.0) {
    throw std::invalid_argument("controller_frequency must be positive and transform_tolerance non-negative");
  }
  control_period_ = 1.0 / controller_frequency;

  pursuit_ = std::make_unique<Pursuit>(config);

  RCLCPP_INFO(
    logger_,
    "Configured %s: lookahead %.2f m, max vel [%.2f %.2f %.2f] m/s %.2f rad/s, "
    "max accel [%.2f %.2f %.2f] m/s^2 %.2f rad/s^2",
    name.c_str(),
    config.lookahead_dist,
    config.max_vel.linear.x(),
    config.max_vel.linear.y(),
    config.max_vel.linear.z(),
    config.max_vel.yaw,
    config.max_accel.linear.x(),
    config.max_accel.linear.y(),
    config.max_accel.linear.z(),
    config.max_accel.yaw);
}

void PurePursuitController3D::cleanup()
{
  pursuit_.reset();
}

void PurePursuitController3D::activate()
{}

void PurePursuitController3D::deactivate()
{}

void PurePursuitController3D::setPlan(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    throw nav2_core::InvalidPath("PurePursuitController3D received an empty path");
  }

  std::vector<Pose4D> poses;
  poses.reserve(path.poses.size());
  for (const auto & pose : path.poses) {
    poses.push_back(to_pose4d(pose.pose));
  }

  path_frame_ = path.header.frame_id;
  pursuit_->set_path(std::move(poses));
}

geometry_msgs::msg::TwistStamped PurePursuitController3D::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  // The measured velocity is not used: the controller server flattens odometry to 2D, dropping
  // heave. As in Orca4, acceleration is limited relative to the previous command instead.
  if (!pursuit_->has_path()) {
    throw nav2_core::InvalidPath("PurePursuitController3D has no path to follow");
  }

  geometry_msgs::msg::PoseStamped pose_in_path_frame;
  if (!nav2_util::transformPoseInTargetFrame(pose, pose_in_path_frame, *tf_, path_frame_, transform_tolerance_)) {
    throw nav2_core::ControllerTFError(
      "Unable to transform the robot pose from '" + pose.header.frame_id + "' to '" + path_frame_ + "'");
  }

  const BodyVelocity command = pursuit_->compute(to_pose4d(pose_in_path_frame.pose), control_period_);

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = base_frame_;
  cmd_vel.header.stamp = pose.header.stamp;
  cmd_vel.twist.linear.x = command.linear.x();
  cmd_vel.twist.linear.y = command.linear.y();
  cmd_vel.twist.linear.z = command.linear.z();
  cmd_vel.twist.angular.z = command.yaw_rate;
  return cmd_vel;
}

void PurePursuitController3D::setSpeedLimit(const double & /*speed_limit*/, const bool & /*percentage*/)
{
  RCLCPP_WARN_ONCE(logger_, "PurePursuitController3D does not support speed limits; ignoring them");
}

void PurePursuitController3D::reset()
{
  pursuit_->clear();
}

}  // namespace rov_nav2_plugins

PLUGINLIB_EXPORT_CLASS(rov_nav2_plugins::PurePursuitController3D, nav2_core::Controller)
