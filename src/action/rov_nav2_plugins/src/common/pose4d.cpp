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

#include "rov_nav2_plugins/common/pose4d.hpp"

#include "nav2_util/geometry_utils.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace rov_nav2_plugins
{

Pose4D to_pose4d(const geometry_msgs::msg::Pose & pose)
{
  return {{pose.position.x, pose.position.y, pose.position.z}, tf2::getYaw(pose.orientation)};
}

geometry_msgs::msg::Pose to_pose_msg(const Pose4D & pose)
{
  geometry_msgs::msg::Pose msg;
  msg.position.x = pose.position.x();
  msg.position.y = pose.position.y();
  msg.position.z = pose.position.z();
  msg.orientation = nav2_util::geometry_utils::orientationAroundZAxis(pose.yaw);
  return msg;
}

}  // namespace rov_nav2_plugins
