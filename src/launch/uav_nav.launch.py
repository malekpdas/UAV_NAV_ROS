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
    rc_control_pkg = get_package_share_directory('rc_control')
    servo_control_pkg = get_package_share_directory('servo_control')
    
    # Config files
    bno085_config = os.path.join(sensors_pkg, 'config', 'bno085_config.yaml')
    gps_config = os.path.join(sensors_pkg, 'config', 'zoe_m8q_config.yaml')
    sensor_fusion_config = os.path.join(ekf_pkg, 'config', 'sensor_fusion_config.yaml')
    rc_control_config = os.path.join(rc_control_pkg, 'config', 'rc_control_config.yaml')
    servo_control_config = os.path.join(servo_control_pkg, 'config', 'servo_control_config.yaml')

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
            parameters=[sensor_fusion_config],
        ),
        
        # ============================================================
        # Control
        # ============================================================
        Node(
            package='rc_control',
            executable='rc_control',
            name='rc_control',
            output='log',
            parameters=[rc_control_config],
        ),
        Node(
            package='servo_control',
            executable='servo_control',
            name='servo_control',
            output='log',
            parameters=[servo_control_config],
        ),
    ])
