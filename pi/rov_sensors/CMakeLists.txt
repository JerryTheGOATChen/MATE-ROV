cmake_minimum_required(VERSION 3.8)
project(rov_sensors)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(bno055_node src/bno055_node.cpp)
target_compile_features(bno055_node PUBLIC cxx_std_17)
ament_target_dependencies(bno055_node rclcpp sensor_msgs std_msgs)
install(TARGETS bno055_node DESTINATION lib/${PROJECT_NAME})

add_executable(bar30_node src/bar30_node.cpp)
target_compile_features(bar30_node PUBLIC cxx_std_17)
ament_target_dependencies(bar30_node rclcpp sensor_msgs std_msgs)
install(TARGETS bar30_node DESTINATION lib/${PROJECT_NAME})

add_executable(leak_sensor_node src/leak_sensor_node.cpp)
target_compile_features(leak_sensor_node PUBLIC cxx_std_17)
ament_target_dependencies(leak_sensor_node rclcpp sensor_msgs std_msgs)
install(TARGETS leak_sensor_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
