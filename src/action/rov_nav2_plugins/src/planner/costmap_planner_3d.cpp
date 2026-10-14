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
#include "rov_nav2_plugins/planner/costmap_planner_3d.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "nav2_core/planner_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rov_nav2_plugins/common/pose4d.hpp"
#include "rov_nav2_plugins/planner/planner_math.hpp"

namespace rov_nav2_plugins
{

CostmapPlanner3D::CostmapPlanner3D()
: loader_("nav2_core", "nav2_core::GlobalPlanner")
{}

CostmapPlanner3D::~CostmapPlanner3D() = default;

void CostmapPlanner3D::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("CostmapPlanner3D: unable to lock the parent node");
  }

  const auto xy_planner_type =
    nav2_util::declare_or_get_parameter(node, name + ".xy_planner", std::string("nav2_smac_planner::SmacPlanner2D"));
  z_before_xy_ = nav2_util::declare_or_get_parameter(node, name + ".z_before_xy", false);

  xy_planner_ = loader_.createUniqueInstance(xy_planner_type);
  xy_planner_->configure(parent, name + ".xy", std::move(tf), costmap_ros);

  RCLCPP_INFO(
    node->get_logger(),
    "Configured %s: xy_planner=%s, z_before_xy=%s",
    name.c_str(),
    xy_planner_type.c_str(),
    z_before_xy_ ? "true" : "false");
}

void CostmapPlanner3D::cleanup()
{
  if (xy_planner_) {
    xy_planner_->cleanup();
  }
  xy_planner_.reset();
}

void CostmapPlanner3D::activate()
{
  xy_planner_->activate();
}

void CostmapPlanner3D::deactivate()
{
  xy_planner_->deactivate();
}

nav_msgs::msg::Path CostmapPlanner3D::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  // The planner server has already transformed start and goal into the costmap's frame. 2D planners
  // ignore z.
  const nav_msgs::msg::Path xy_plan = xy_planner_->createPlan(start, goal, cancel_checker);
  if (xy_plan.poses.empty()) {
    throw nav2_core::NoValidPathCouldBeFound("The 2D planner returned an empty path");
  }

  std::vector<Eigen::Vector2d> xy_path;
  xy_path.reserve(xy_plan.poses.size());
  for (const auto & pose : xy_plan.poses) {
    xy_path.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }

  nav_msgs::msg::Path path;
  path.header = xy_plan.header;

  const auto poses = add_depth(xy_path, to_pose4d(start.pose), to_pose4d(goal.pose), z_before_xy_);
  path.poses.reserve(poses.size());
  for (const Pose4D & pose : poses) {
    geometry_msgs::msg::PoseStamped stamped;
    stamped.header = path.header;
    stamped.pose = to_pose_msg(pose);
    path.poses.push_back(stamped);
  }

  return path;
}

}  // namespace rov_nav2_plugins

PLUGINLIB_EXPORT_CLASS(rov_nav2_plugins::CostmapPlanner3D, nav2_core::GlobalPlanner)
