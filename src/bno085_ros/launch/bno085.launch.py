import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('bno085_ros')
    config = os.path.join(pkg_share, 'config', 'bno085_config.yaml')

    return LaunchDescription([
        Node(
            package='bno085_ros',
            executable='bno085_node',
            name='imu_bno085',
            output='log',
            parameters=[config],
        ),
    ])
