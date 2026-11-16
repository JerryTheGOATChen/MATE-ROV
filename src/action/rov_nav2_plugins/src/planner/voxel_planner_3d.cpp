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
#include "rov_nav2_plugins/planner/voxel_planner_3d.hpp"

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "nav2_core/planner_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rov_nav2_plugins/common/pose4d.hpp"

namespace rov_nav2_plugins
{

void VoxelPlanner3D::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("VoxelPlanner3D: unable to lock the parent node");
  }
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  global_frame_ = costmap_ros->getGlobalFrameID();

  const auto param = [&node, &name](const std::string & param_name, auto default_value) {
    return nav2_util::declare_or_get_parameter(node, name + "." + param_name, default_value);
  };
  const auto layer_name = param("voxel_layer", std::string("voxel_layer_3d"));
  config_.cost_weight = param("cost_weight", 2.0);
  const int unknown_cost = param("unknown_cost", 40);
  config_.z_weight = param("z_weight", 1.5);
  config_.max_planning_time = param("max_planning_time", 1.0);
  const auto yaw_mode = param("yaw_mode", std::string("interpolate"));

  if (config_.cost_weight < 0.0 || config_.z_weight <= 0.0 || config_.max_planning_time <= 0.0) {
    throw std::invalid_argument(
      name + ": cost_weight must be non-negative, and z_weight and max_planning_time positive");
  }
  if (unknown_cost < kFreeCost || unknown_cost >= kInscribedCost) {
    throw std::invalid_argument(name + ": unknown_cost must be from 0 to 252");
  }
  config_.unknown_cost = static_cast<std::uint8_t>(unknown_cost);
  if (yaw_mode == "interpolate") {
    yaw_mode_ = YawMode::kInterpolate;
  } else if (yaw_mode == "face_travel") {
    yaw_mode_ = YawMode::kFaceTravel;
  } else {
    throw std::invalid_argument(name + ": yaw_mode must be \"interpolate\" or \"face_travel\"");
  }

  // The planner server configures its costmap, loading the layers, before its planners
  for (const auto & layer : *costmap_ros->getLayeredCostmap()->getPlugins()) {
    if (layer->getName() == layer_name) {
      layer_ = std::dynamic_pointer_cast<VoxelLayer3D>(layer);
    }
  }
  if (!layer_) {
    throw std::runtime_error(
      name + ": the costmap '" + costmap_ros->get_name() + "' has no VoxelLayer3D named '" + layer_name + "'");
  }

  RCLCPP_INFO(
    logger_,
    "Configured %s: voxel_layer=%s, cost_weight=%.2f, unknown_cost=%d, z_weight=%.2f, yaw_mode=%s",
    name.c_str(),
    layer_name.c_str(),
    config_.cost_weight,
    unknown_cost,
    config_.z_weight,
    yaw_mode.c_str());
}

void VoxelPlanner3D::cleanup()
{
  layer_.reset();
}

void VoxelPlanner3D::activate()
{}

void VoxelPlanner3D::deactivate()
{}

nav_msgs::msg::Path VoxelPlanner3D::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  // The planner server has already transformed start and goal into the costmap's frame
  const auto volume = layer_->cost_volume();
  if (!volume) {
    throw nav2_core::PlannerException("VoxelPlanner3D: the voxel layer's grid has no size");
  }

  const Pose4D start_pose = to_pose4d(start.pose);
  const Pose4D goal_pose = to_pose4d(goal.pose);
  const Plan3D plan = plan_through_voxels(*volume, start_pose.position, goal_pose.position, config_, cancel_checker);
  RCLCPP_DEBUG(logger_, "VoxelPlanner3D expanded %zu voxels", plan.expansions);

  switch (plan.status) {
    case PlanStatus::kSucceeded:
      break;
    case PlanStatus::kStartOutside:
      throw nav2_core::StartOutsideMapBounds("The start is outside the voxel grid");
    case PlanStatus::kGoalOutside:
      throw nav2_core::GoalOutsideMapBounds("The goal is outside the voxel grid");
    case PlanStatus::kStartOccupied:
      throw nav2_core::StartOccupied("The start is in an obstacle");
    case PlanStatus::kGoalOccupied:
      throw nav2_core::GoalOccupied("The goal is within the vehicle's size of an obstacle");
    case PlanStatus::kNoPath:
      throw nav2_core::NoValidPathCouldBeFound("Obstacles block every route to the goal");
    case PlanStatus::kTimedOut:
      throw nav2_core::PlannerTimedOut("No route found within max_planning_time");
    case PlanStatus::kCanceled:
      throw nav2_core::PlannerCancelled("Planning was canceled");
  }

  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = clock_->now();
  for (const Pose4D & pose : add_yaw(plan.points, start_pose.yaw, goal_pose.yaw, yaw_mode_)) {
    geometry_msgs::msg::PoseStamped stamped;
    stamped.header = path.header;
    stamped.pose = to_pose_msg(pose);
    path.poses.push_back(stamped);
  }
  return path;
}

}  // namespace rov_nav2_plugins

PLUGINLIB_EXPORT_CLASS(rov_nav2_plugins::VoxelPlanner3D, nav2_core::GlobalPlanner)
