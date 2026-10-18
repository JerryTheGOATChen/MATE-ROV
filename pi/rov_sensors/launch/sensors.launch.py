from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', description='Absolute path to a ROS 2 parameter YAML file'),
        Node(
          package='rov_sensors',
          executable='bno055_node',
          name='bno055_node',
          output='screen',
          parameters=[params_file]
          ),
        Node(
          package='rov_sensors',
          executable='bar30_node',
          name='bar30_node',
          output='screen',
          parameters=[params_file]
          ),
        Node(
          package='rov_sensors',
          executable='leak_sensor_node',
          name='leak_sensor_node',
          output='screen',
          parameters=[params_file]
          ),
    ])
