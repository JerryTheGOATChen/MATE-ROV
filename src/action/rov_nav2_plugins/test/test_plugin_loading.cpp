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

#include <string>

#include <catch2/catch_test_macros.hpp>

#include "nav2_core/controller.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_core/progress_checker.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "pluginlib/class_loader.hpp"

namespace
{

/// Load `plugin` the way the Nav2 servers do: by name, through the ament index entries of `package`.
template <typename Base>
bool loads(const std::string & package, const std::string & base_class, const std::string & plugin)
{
  pluginlib::ClassLoader<Base> loader(package, base_class);
  return loader.createUniqueInstance(plugin) != nullptr;
}

}  // namespace

TEST_CASE("Nav2 can load every plugin by name", "[plugins]")
{
  REQUIRE(loads<nav2_core::GlobalPlanner>("nav2_core", "nav2_core::GlobalPlanner", "rov_nav2_plugins::VoxelPlanner3D"));
  REQUIRE(
    loads<nav2_core::Controller>("nav2_core", "nav2_core::Controller", "rov_nav2_plugins::PurePursuitController3D"));
  REQUIRE(loads<nav2_core::GoalChecker>("nav2_core", "nav2_core::GoalChecker", "rov_nav2_plugins::GoalChecker3D"));
  REQUIRE(loads<nav2_core::ProgressChecker>(
    "nav2_core", "nav2_core::ProgressChecker", "rov_nav2_plugins::ProgressChecker3D"));
  REQUIRE(loads<nav2_costmap_2d::Layer>("nav2_costmap_2d", "nav2_costmap_2d::Layer", "rov_nav2_plugins::VoxelLayer3D"));
}
