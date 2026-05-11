from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('save_dir',    default_value='/tmp/eos_slam_maps'),
        DeclareLaunchArgument('map_name',    default_value='eos_map'),
        DeclareLaunchArgument('auto_save_s', default_value='0.0',
                              description='Auto-save interval in seconds (0=disabled)'),

        Node(
            package='eos_slam',
            executable='map_saver_node',
            name='eos_map_saver',
            output='screen',
            parameters=[{
                'save_dir':    LaunchConfiguration('save_dir'),
                'map_name':    LaunchConfiguration('map_name'),
                'auto_save_s': LaunchConfiguration('auto_save_s'),
            }],
        ),
    ])
