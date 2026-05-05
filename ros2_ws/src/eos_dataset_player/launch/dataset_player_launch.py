#!/usr/bin/env python3
"""
Launch file for eos_dataset_player.

Usage:
  ros2 launch eos_dataset_player dataset_player_launch.py dataset_path:=/path/to/MH_01_easy
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'dataset_path',
            description='Path to EuRoC dataset root',
        ),
        DeclareLaunchArgument(
            'replay_rate',
            default_value='30.0',
            description='Replay rate in Hz',
        ),
        Node(
            package='eos_dataset_player',
            executable='dataset_player',
            name='dataset_player',
            output='screen',
            parameters=[{
                'dataset_path': LaunchConfiguration('dataset_path'),
                'replay_rate': LaunchConfiguration('replay_rate'),
            }],
        ),
    ])
