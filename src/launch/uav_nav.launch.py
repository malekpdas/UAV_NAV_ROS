"""
Global Launch File for UAV Navigation Stack
============================================
Launches all UAV components:
- IMU (BNO085)
- GPS (ZOE-M8Q)
- EKF State Estimator
- RC Control Input
- Servo/ESC Control Output
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    sensors_pkg = get_package_share_directory('sensors')
    ekf_pkg = get_package_share_directory('sensor_fusion')
    rc_pkg = get_package_share_directory('rc_control_ros')
    servo_pkg = get_package_share_directory('servo_control_ros')
    
    # Config files
    bno085_config = os.path.join(sensors_pkg, 'config', 'bno085_config.yaml')
    gps_config = os.path.join(sensors_pkg, 'config', 'zoe_m8q_config.yaml')
    ekf_config = os.path.join(ekf_pkg, 'config', 'ekf_config.yaml')
    rc_config = os.path.join(rc_pkg, 'config', 'rc_params.yaml')
    servo_config = os.path.join(servo_pkg, 'config', 'servo_params.yaml')

    return LaunchDescription([
        # ============================================================
        # Sensors
        # ============================================================
        Node(
            package='sensors',
            executable='imu_bno085',
            name='imu_bno085',
            output='log',
            parameters=[bno085_config],
        ),
        Node(
            package='sensors',
            executable='gps_zoe_m8q',
            name='gps_zoe_m8q',
            output='log',
            parameters=[gps_config],
        ),
        
        # ============================================================
        # State Estimation
        # ============================================================
        Node(
            package='sensor_fusion',
            executable='sensor_fusion',
            name='sensor_fusion',
            output='log',
            parameters=[ekf_config],
        ),
        
        # ============================================================
        # Control
        # ============================================================
        Node(
            package='rc_control_ros',
            executable='rc_reader_node',
            name='rc_reader_node',
            output='log',
            parameters=[rc_config],
        ),
        Node(
            package='servo_control_ros',
            executable='servo_controller_node',
            name='servo_controller_node',
            output='log',
            parameters=[servo_config],
        ),
    ])
