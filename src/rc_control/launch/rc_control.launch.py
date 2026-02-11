import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('rc_control')
    config_file = os.path.join(pkg_share, 'config', 'rc_control.yaml')

    return LaunchDescription([
        Node(
            package='rc_control',
            executable='rc_control',
            name='rc_control',
            output='log',
            parameters=[config_file]
        )
    ])
