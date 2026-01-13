import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('rc_control_ros')
    config_file = os.path.join(pkg_share, 'config', 'rc_params.yaml')

    return LaunchDescription([
        Node(
            package='rc_control_ros',
            executable='rc_reader_node',
            name='rc_reader_node',
            output='screen',
            parameters=[config_file]
        )
    ])
