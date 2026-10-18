from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', description='Absolute path to a ROS 2 parameter YAML file'),
        Node(
          package='rov_cameras',
          executable='front_camera_node',
          name='front_camera_node',
          output='screen',
          parameters=[params_file]
          ),
        Node(
          package='rov_cameras',
          executable='down_camera_node',
          name='down_camera_node',
          output='screen',
          parameters=[params_file]
          ),
    ])
