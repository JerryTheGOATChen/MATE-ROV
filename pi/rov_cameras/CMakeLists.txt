cmake_minimum_required(VERSION 3.8)
project(rov_cameras)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(front_camera_node src/front_camera_node.cpp)
target_compile_features(front_camera_node PUBLIC cxx_std_17)
ament_target_dependencies(front_camera_node rclcpp sensor_msgs)
install(TARGETS front_camera_node DESTINATION lib/${PROJECT_NAME})

add_executable(down_camera_node src/down_camera_node.cpp)
target_compile_features(down_camera_node PUBLIC cxx_std_17)
ament_target_dependencies(down_camera_node rclcpp sensor_msgs)
install(TARGETS down_camera_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
