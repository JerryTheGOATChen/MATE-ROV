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
project(rov_kinematic_sim)

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
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ----------------------------
# Component library
# ----------------------------
add_library(kinematic_sim_component SHARED
  src/kinematic_sim_node.cpp
  src/rov_kinematics.cpp
)

target_include_directories(kinematic_sim_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(kinematic_sim_component
  PUBLIC
    Eigen3::Eigen
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    tf2_ros::tf2_ros
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
  PRIVATE
    angles::angles
    rclcpp_components::component
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
)

rclcpp_components_register_node(
  kinematic_sim_component
  PLUGIN "rov_kinematic_sim::KinematicSimNode"
  EXECUTABLE kinematic_sim_node
)

# ----------------------------
# Install
# ----------------------------
install(
  TARGETS kinematic_sim_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch missions
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(wato_test REQUIRED)
  find_package(lifecycle_msgs REQUIRED)
  find_package(tf2_msgs REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  add_wato_test(test_rov_kinematics test/test_rov_kinematics.cpp LIBRARIES kinematic_sim_component)
  add_wato_test(test_kinematic_sim_node test/test_kinematic_sim_node.cpp
    LIBRARIES
      kinematic_sim_component
      ${lifecycle_msgs_TARGETS}
      ${tf2_msgs_TARGETS}
  )

  # Flies the acceptance missions through the full Nav2 stack on this simulator. It runs in its own
  # ROS domain so it cannot cross-talk with other tests, or with a stack running in a DevContainer.
  add_launch_test(test/test_sim_acceptance.py
    TIMEOUT 300
    ENV ROS_DOMAIN_ID=98
  )
endif()

ament_package()
