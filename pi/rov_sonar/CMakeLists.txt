cmake_minimum_required(VERSION 3.8)
project(rov_sonar)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(ping2_node src/ping2_node.cpp)
target_compile_features(ping2_node PUBLIC cxx_std_17)
ament_target_dependencies(ping2_node rclcpp sensor_msgs)
install(TARGETS ping2_node DESTINATION lib/${PROJECT_NAME})

add_executable(ping360_node src/ping360_node.cpp)
target_compile_features(ping360_node PUBLIC cxx_std_17)
ament_target_dependencies(ping360_node rclcpp sensor_msgs)
install(TARGETS ping360_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
