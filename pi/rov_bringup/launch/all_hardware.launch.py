from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([FindPackageShare('rov_bringup'), 'config', 'publish_rates.yaml']),
            description='Shared ROS 2 parameter YAML file'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('rov_sensors'), 'launch', 'sensors.launch.py'])),
            launch_arguments={'params_file': params_file}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('rov_sonar'), 'launch', 'sonars.launch.py'])),
            launch_arguments={'params_file': params_file}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('rov_cameras'), 'launch', 'cameras.launch.py'])),
            launch_arguments={'params_file': params_file}.items()
        ),
    ])
