from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'bridge_tf',
            default_value='true',
            description='Relay /tf and /tf_static topics'),
        DeclareLaunchArgument(
            'status_period',
            default_value='5.0',
            description='Status publish period in seconds'),

        Node(
            package='eos_sensor_bridge',
            executable='sensor_bridge',
            name='eos_sensor_bridge',
            output='screen',
            parameters=[{
                'bridge_tf': LaunchConfiguration('bridge_tf'),
                'status_period': LaunchConfiguration('status_period'),
            }],
        ),
    ])
