from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('odom_frame',      default_value='odom'),
        DeclareLaunchArgument('base_frame',      default_value='base_link'),
        DeclareLaunchArgument('publish_rate_hz', default_value='20.0'),

        Node(
            package='eos_slam',
            executable='nav2_bridge_node',
            name='eos_nav2_bridge',
            output='screen',
            parameters=[{
                'odom_frame':      LaunchConfiguration('odom_frame'),
                'base_frame':      LaunchConfiguration('base_frame'),
                'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
            }],
        ),
    ])
