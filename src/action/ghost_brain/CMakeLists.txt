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
project(ghost_brain)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ----------------------------
# Dependencies
# ----------------------------
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(haunted_house_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# ----------------------------
# Component library
# ----------------------------
# Add any new .cpp files you write to this list
add_library(ghost_brain_component SHARED
  src/controller.cpp
  src/ghost_brain_node.cpp
  src/localizer.cpp
  src/planner.cpp
  src/tracker.cpp
)

target_include_directories(ghost_brain_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(ghost_brain_component
  PUBLIC
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    tf2::tf2
    tf2_ros::tf2_ros
    ${geometry_msgs_TARGETS}
    ${haunted_house_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${vision_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
  PRIVATE
    rclcpp_components::component
)

rclcpp_components_register_node(
  ghost_brain_component
  PLUGIN "ghost_brain::GhostBrainNode"
  EXECUTABLE ghost_brain_node
)

# ----------------------------
# Install
# ----------------------------
install(
  TARGETS ghost_brain_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(wato_test REQUIRED)
  find_package(lifecycle_msgs REQUIRED)

  add_wato_test(test_planning test/test_planning.cpp LIBRARIES ghost_brain_component)
  add_wato_test(test_sensing test/test_sensing.cpp LIBRARIES ghost_brain_component)
  add_wato_test(test_ghost_brain_node test/test_ghost_brain_node.cpp
    LIBRARIES
      ghost_brain_component
      ${lifecycle_msgs_TARGETS}
  )
endif()

ament_package()
