"""Main launch file for the basekit project."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Generate launch description for the complete basekit system."""
    pkg_dir = get_package_share_directory('basekit_launch')
    ui_pkg_dir = get_package_share_directory('example_ui')

    # Include the GNSS launch file
    gnss_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'gnss.launch.py')
        )
    )

    # Include the field friend launch file
    field_friend_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'field_friend.launch.py')
        )
    )

    # Include the camera system launch file
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'camera_system.launch.py')
        )
    )

    # Include the UI launch file
    ui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ui_pkg_dir, 'launch', 'ui.launch.py')
        )
    )

    return LaunchDescription([
        gnss_launch,
        field_friend_launch,
        camera_launch,
        ui_launch
    ])
