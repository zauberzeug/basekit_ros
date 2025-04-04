import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    basekit_ui_node = Node(
        package='basekit',
        executable='basekit_ui',
        name='basekit_ui',
    )

    basekit_simulator_node = Node(
        package='basekit',
        executable='basekit_simulator',
        name='basekit_simulator',
    )

    foxglove_launch_include = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('foxglove_bridge'),
                'launch/foxglove_bridge_launch.xml',
            )
        )
    )

    return LaunchDescription(
        [
            # DeclareLaunchArgument(
            #     "rviz", default_value="false", description="Open RViz."
            # ),
            basekit_ui_node,
            basekit_simulator_node,
            foxglove_launch_include,
        ]
    )
