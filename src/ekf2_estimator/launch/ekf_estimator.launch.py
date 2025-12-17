import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('ekf2_estimator'),
        'config',
        'ekf2_params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_link_frame', default_value='base_link'),

        Node(
            package='ekf2_estimator',
            executable='ekf_node',
            name='ekf2_node',
            output='screen',
            parameters=[config, {
                'map_frame': LaunchConfiguration('map_frame'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'base_link_frame': LaunchConfiguration('base_link_frame')
            }]
        )
    ])
