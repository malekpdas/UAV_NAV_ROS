import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('zoe_m8q_ros')
    config = os.path.join(pkg_share, 'config', 'zoe_m8q_config.yaml')

    return LaunchDescription([
        Node(
            package='zoe_m8q_ros',
            executable='zoe_m8q_node',
            name='gps_zoe_m8q',
            output='log',
            parameters=[config],
        ),
    ])
