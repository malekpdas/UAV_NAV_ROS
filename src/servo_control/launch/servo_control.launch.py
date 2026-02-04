import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('servo_control')
    config_file = os.path.join(pkg_share, 'config', 'servo_control_config.yaml')

    return LaunchDescription([
        Node(
            package='servo_control',
            executable='servo_control',
            name='servo_control',
            output='log',
            parameters=[config_file]
        )
    ])
