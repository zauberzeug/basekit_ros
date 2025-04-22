"""Launch file for the Field Friend driver node.

Note: The Lizard startup files (startup.liz and ff_basic_startup.liz) are located in the
config directory and should be accessible by the field_friend_driver node. These files
contain the hardware configuration and initialization for the Field Friend robot's lizard.
These are not changeable right now, but will be in the future. Right now it will use the
startup.liz file in the field_friend_driver package.
"""

import os

import ament_index_python.packages
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Field Friend driver."""
    config_file = LaunchConfiguration('config_file')

    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory(
            'basekit_launch'),
        'config')
    config_file_launch_arg = DeclareLaunchArgument(
        'config_file', default_value=os.path.join(config_directory, 'field_friend.yaml')
    )

    return LaunchDescription([
        config_file_launch_arg,
        Node(
            package='field_friend_driver',
            executable='field_friend_driver_node',
            parameters=[config_file],
            respawn=True,
            respawn_delay=5,
            name='controller'
        )
    ])
