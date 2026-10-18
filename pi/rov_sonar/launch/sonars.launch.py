from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', description='Absolute path to a ROS 2 parameter YAML file'),
        Node(
          package='rov_sonar',
          executable='ping2_node',
          name='ping2_node',
          output='screen',
          parameters=[params_file]
          ),
        Node(
          package='rov_sonar',
          executable='ping360_node',
          name='ping360_node',
          output='screen',
          parameters=[params_file]
          ),
    ])
