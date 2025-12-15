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
    
    gps_config = os.path.join(
        get_package_share_directory('gps_zoe_m8q_driver'),
        'config',
        'params.yaml'
    )
    
    ekf_config = os.path.join(
        get_package_share_directory('ekf2_estimator'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='imu_bmx160_driver',
            executable='imu_bmx160_driver_node',
            name='imu_driver',
            parameters=[imu_config]
        ),
        Node(
            package='gps_zoe_m8q_driver',
            executable='gps_zoe_m8q_node',
            name='gps_driver',
            parameters=[gps_config]
        ),
        Node(
            package='ekf2_estimator',
            executable='ekf2_node',
            name='ekf_estimator',
            parameters=[ekf_config]
        )
    ])
