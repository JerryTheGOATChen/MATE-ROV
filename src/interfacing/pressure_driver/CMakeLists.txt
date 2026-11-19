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
project(pressure_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(bar30_component SHARED
  src/bar30_node.cpp
  src/ms5837.cpp
)

target_include_directories(bar30_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(bar30_component
  PUBLIC
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    ${nav_msgs_TARGETS}
  PRIVATE
    rclcpp_components::component
)

rclcpp_components_register_node(
  bar30_component
  PLUGIN "pressure_driver::Bar30Node"
  EXECUTABLE bar30_node
)

install(
  TARGETS bar30_component
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

  add_wato_test(test_ms5837 test/test_ms5837.cpp LIBRARIES bar30_component)
  add_wato_test(test_bar30_node test/test_bar30_node.cpp
    LIBRARIES
      bar30_component
      ${lifecycle_msgs_TARGETS}
  )
endif()

ament_package()
