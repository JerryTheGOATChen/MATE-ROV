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
project(altimeter_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(ping2_component SHARED
  src/ping2_node.cpp
  src/ping2.cpp
)

target_include_directories(ping2_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(ping2_component
  PUBLIC
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    ${sensor_msgs_TARGETS}
  PRIVATE
    rclcpp_components::component
)

rclcpp_components_register_node(
  ping2_component
  PLUGIN "altimeter_driver::Ping2Node"
  EXECUTABLE ping2_node
)

install(
  TARGETS ping2_component
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
  find_package(lifecycle_msgs REQUIRED)

  add_wato_test(test_ping2 test/test_ping2.cpp LIBRARIES ping2_component)
  add_wato_test(test_ping2_node test/test_ping2_node.cpp
    LIBRARIES
      ping2_component
      ${lifecycle_msgs_TARGETS}
  )
endif()

ament_package()
