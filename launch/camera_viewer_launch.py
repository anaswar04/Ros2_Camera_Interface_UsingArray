import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('camera_viewer')
    config_file = os.path.join(pkg_share, 'config', 'cameras.yaml')

    return LaunchDescription([
        Node(
            package='camera_viewer',
            executable='camera_publisher',
            name='camera_viewer',
            parameters=[config_file]
        )
    ])
