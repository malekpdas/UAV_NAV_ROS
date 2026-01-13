from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('bno085_ros')
    config_file = os.path.join(pkg_share, 'config', 'bno085.yaml')

    return LaunchDescription([
        Node(
            package='bno085_ros',
            executable='bno085_node',
            name='bno085_node',
            output='screen',
            parameters=[config_file],
            emulate_tty=True
        )
    ])
