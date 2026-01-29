import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('ekf_estimator')
    config = os.path.join(pkg_share, 'config', 'ekf_params.yaml')

    return LaunchDescription([
        Node(
            package='ekf_estimator',
            executable='ekf_node',
            name='ekf_node',
            output='log',
            parameters=[config],
        ),
    ])
