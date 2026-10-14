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
project(rov_gazebo_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch missions models worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)

  # Flies the pool missions through Nav2 in Gazebo. Its own ROS domain and Gazebo partition keep it
  # from cross-talking with other tests or a running simulation.
  add_launch_test(test/test_pool_missions.py
    TIMEOUT 400
    ENV ROS_DOMAIN_ID=97 GZ_PARTITION=rov_gazebo_sim_test
  )
endif()

ament_package()
