import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_lite_ros')
    config = os.path.join(pkg_share, 'config', 'lidar_lite_config.yaml')

    return LaunchDescription([
        Node(
            package='lidar_lite_ros',
            executable='lidar_lite_node',
            name='lidar_lite_node',
            output='log',
            parameters=[config],
        ),
    ])
