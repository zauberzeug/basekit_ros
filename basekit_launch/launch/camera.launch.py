"""Launch file for the USB camera with Foxglove Bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for USB camera and Foxglove Bridge."""

    # Get the configuration file path
    config_file = os.path.join(
        get_package_share_directory('basekit_launch'),
        'config',
        'camera.yaml'
    )

    return LaunchDescription([
        # Camera Node
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[config_file],
            remappings=[
                ('image_raw', '/field_friend/camera/image_raw'),
                ('camera_info', '/field_friend/camera/camera_info'),
            ]
        ),

        # Foxglove Bridge Node
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{
                'port': 8765,
                'address': '0.0.0.0',  # Allow external connections
                'tls': False,
                'compression': 'JPEG',  # Use JPEG compression for images
                'max_qos_latency': 0.1,  # 100ms latency for real-time viewing
            }],
        )
    ])
