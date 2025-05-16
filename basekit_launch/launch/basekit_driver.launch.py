"""Launch file for the basekit driver node."""

import os

import ament_index_python.packages
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Basekit driver."""
    config_file = LaunchConfiguration('config_file')
    startup_file = LaunchConfiguration('startup_file')

    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory(
            'basekit_launch'),
        'config')

    config_file_launch_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(config_directory, 'basekit.yaml')
    )

    startup_file_launch_arg = DeclareLaunchArgument(
        'startup_file',
        default_value=os.path.join(config_directory, 'basekit.liz'),
        description='Path to the Lizard startup file (basekit.liz)'
    )

    return LaunchDescription([
        config_file_launch_arg,
        startup_file_launch_arg,
        Node(
            package='basekit_driver',
            executable='basekit_driver_node',
            name='controller',
            parameters=[
                config_file,
                {
                    'startup_file': startup_file,
                }
            ],
            respawn=True,
            respawn_delay=5,
        )
    ])
