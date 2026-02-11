import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('sensors')
    config = os.path.join(pkg_share, 'config', 'imu_bno085.yaml')

    return LaunchDescription([
        Node(
            package='sensors',
            executable='imu_bno085',
            name='imu_bno085',
            output='log',
            parameters=[config],
        ),
    ])
