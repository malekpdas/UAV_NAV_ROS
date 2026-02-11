import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('sensor_fusion')
    config = os.path.join(pkg_share, 'config', 'sensor_fusion.yaml')

    return LaunchDescription([
        Node(
            package='sensor_fusion',
            executable='sensor_fusion',
            name='sensor_fusion',
            output='log',
            parameters=[config],
        ),
    ])
