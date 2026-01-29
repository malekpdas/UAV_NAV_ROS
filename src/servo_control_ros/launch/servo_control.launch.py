import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('servo_control_ros')
    config_file = os.path.join(pkg_share, 'config', 'servo_params.yaml')

    return LaunchDescription([
        Node(
            package='servo_control_ros',
            executable='servo_controller_node',
            name='servo_controller_node',
            output='log',
            parameters=[config_file]
        )
    ])
