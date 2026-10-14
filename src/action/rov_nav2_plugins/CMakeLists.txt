# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
cmake_minimum_required(VERSION 3.8)
project(rov_nav2_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ----------------------------
# Dependencies
# ----------------------------
find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ----------------------------
# Plugin library
# ----------------------------
add_library(${PROJECT_NAME} SHARED
  src/common/pose4d.cpp
  src/controller/controller_math.cpp
  src/controller/pure_pursuit_controller_3d.cpp
  src/goal_checker/goal_checker_3d.cpp
  src/planner/costmap_planner_3d.cpp
  src/planner/planner_math.cpp
  src/progress_checker/progress_checker_3d.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# nav2_core is header-only and exports include directories rather than a target
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${nav2_core_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    Eigen3::Eigen
    nav2_costmap_2d::nav2_costmap_2d_core
    nav2_util::nav2_util_core
    pluginlib::pluginlib
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    tf2_ros::tf2_ros
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
  PRIVATE
    angles::angles
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
)

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

# ----------------------------
# Install
# ----------------------------
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(wato_test REQUIRED)

  # Algorithms, independent of Nav2 and ROS communication
  add_wato_test(test_controller_math test/controller/test_controller_math.cpp LIBRARIES ${PROJECT_NAME})
  add_wato_test(test_planner_math test/planner/test_planner_math.cpp LIBRARIES ${PROJECT_NAME})

  # Nav2 plugin interfaces
  add_wato_test(test_costmap_planner_3d test/planner/test_costmap_planner_3d.cpp LIBRARIES ${PROJECT_NAME})
  add_wato_test(test_pure_pursuit_controller_3d test/controller/test_pure_pursuit_controller_3d.cpp LIBRARIES ${PROJECT_NAME})
  add_wato_test(test_goal_checker_3d test/goal_checker/test_goal_checker_3d.cpp LIBRARIES ${PROJECT_NAME})
  add_wato_test(test_progress_checker_3d test/progress_checker/test_progress_checker_3d.cpp LIBRARIES ${PROJECT_NAME})

  # Loads every plugin by name through pluginlib, as the Nav2 servers do. Deliberately not linked
  # against the plugin library, so it checks plugins.xml and the export macros.
  add_wato_test(test_plugin_loading test/test_plugin_loading.cpp
    LIBRARIES
      nav2_costmap_2d::nav2_costmap_2d_core
      nav2_util::nav2_util_core
      pluginlib::pluginlib
  )
  target_include_directories(test_plugin_loading SYSTEM PRIVATE ${nav2_core_INCLUDE_DIRS})
endif()

ament_package()
