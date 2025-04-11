""" Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
"""

import os

import ament_index_python.packages
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    config_file = LaunchConfiguration('config_file')

    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory(
            'field_friend_driver'),
        'config')
    config_file_launch_arg = DeclareLaunchArgument(
        'config_file', default_value=os.path.join(config_directory, 'u6.yaml')
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
