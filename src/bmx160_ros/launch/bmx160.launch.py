import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('bmx160_ros'),
        'config',
        'config.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('bus', default_value='1', description='I2C Bus Number'),
        DeclareLaunchArgument('frame_id', default_value='imu_link', description='Frame ID for header'),
        
        Node(
            package='bmx160_ros',
            executable='bmx160_node',
            name='imu_bmx160',
            output='screen',
            parameters=[config, {
                'bus': LaunchConfiguration('bus'),
                'frame_id': LaunchConfiguration('frame_id')
            }]
        )
    ])
