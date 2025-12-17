from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    
    # 1. IMU (BMX160)
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bmx160_ros'),
                'launch',
                'bmx160.launch.py'
            ])
        ])
    )
    
    # 2. GPS (Zoe M8Q)
    gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zoe_m8q_ros'),
                'launch',
                'zoe_m8q.launch.py'
            ])
        ])
    )
    
    # 3. EKF2 Estimator (Delayed slightly to allow sensors to start)
    # Although EKF waits for data anyway, good practice.
    ekf_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('ekf2_estimator'),
                        'launch',
                        'ekf_estimator.launch.py'
                    ])
                ])
            )
        ]
    )

    return LaunchDescription([
        imu_launch,
        gps_launch,
        ekf_launch
    ])
