"""Launch file for AXIS cameras."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for AXIS cameras."""

    basekit_launch_dir = get_package_share_directory('basekit_launch')
    axis_config = os.path.join(basekit_launch_dir, 'config', 'axis_camera.yaml')

    # Define the AXIS camera addresses as launch arguments
    axis_cameras = {
        'camera1': '192.168.42.3',
        'camera2': '192.168.42.3',
        'camera3': '192.168.42.3',
        'camera4': '192.168.42.3',
        'camera5': '192.168.42.3',  # multi-camera
    }

    nodes = []

    # Create nodes for each AXIS camera
    for camera_name, address in axis_cameras.items():
        camera_number = int(camera_name[-1])
        namespace = f'axis_{camera_name}'
        nodes.append(
            Node(
                package='axis_camera',
                executable='axis_camera_node',
                namespace=namespace,
                name='camera',
                parameters=[
                    axis_config,
                    {
                        'hostname': address,
                        'http_port': 80,
                        'username': 'root',
                        'password': 'zauberzg!',
                        'use_encrypted_password': False,
                        'frame_id': f'{namespace}/camera',
                        'camera_info_url': '',
                        'stream': 'mjpeg',
                        'camera': camera_number,
                        'height': 480,
                        'width': 640,
                        'fps': 20,
                        'tf_prefix': camera_name,
                        'ir': False,
                        'wiper': False,
                        'defog': False,
                        'ptz': False,
                        'ptz_teleop': False
                    }
                ],
            )
        )

    return LaunchDescription(nodes)
