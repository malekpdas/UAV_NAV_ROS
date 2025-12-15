from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gps_config = os.path.join(
        get_package_share_directory('gps_zoe_m8q_driver'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='gps_zoe_m8q_driver',
            executable='gps_zoe_m8q_node',
            name='gps_driver',
            parameters=[gps_config]
        )
    ])
