import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('calibration')
    config = os.path.join(pkg_share, 'config', 'mag_calibration.yaml')

    return LaunchDescription([
        Node(
            package='calibration',
            executable='mag_calibration',
            name='mag_calibration',
            output='log',
            parameters=[config],
        )
    ])
