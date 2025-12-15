from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    imu_config = os.path.join(
        get_package_share_directory('imu_bmx160_driver'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='imu_bmx160_driver',
            executable='imu_bmx160_driver_node',
            name='imu_driver',
            parameters=[imu_config]
        )
    ])
