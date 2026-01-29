import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('bmx160_ros')
    config = os.path.join(pkg_share, 'config', 'bmx160_config.yaml')

    return LaunchDescription([
        Node(
            package='bmx160_ros',
            executable='bmx160_node',
            name='imu_bmx160',
            output='log',
            parameters=[config],
        )
    ])
