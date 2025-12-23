from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    pkg_name = 'lidar_lite_ros'
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='config.yaml',
        description='Name of the config file to load'
    )
    
    config = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'config',
        LaunchConfiguration('config_file')
    ])

    return LaunchDescription([
        config_file_arg,
        DeclareLaunchArgument('i2c_bus', default_value='1', description='I2C Bus Number'),
        DeclareLaunchArgument('frequency', default_value='50.0', description='Publishing frequency (Hz)'),
        DeclareLaunchArgument('frame_id', default_value='lidar_link', description='Frame ID for header'),
        DeclareLaunchArgument('preset', default_value='balanced', description='Hardware preset (balanced, high_speed, high_accuracy, long_range)'),
        DeclareLaunchArgument('sig_count_val', default_value='-1', description='Manual override for signal count'),
        
        Node(
            package=pkg_name,
            executable='lidar_lite_node',
            name='lidar_lite_node',
            output='screen',
            parameters=[config]
        )
    ])
