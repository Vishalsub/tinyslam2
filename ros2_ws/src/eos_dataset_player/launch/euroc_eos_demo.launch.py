#!/usr/bin/env python3
"""
One-shot EuRoC → EOS-SLAM → RViz demo.

RViz alone shows nothing: it subscribes to /eos/... topics that only exist when
dataset + bridge + eos_slam nodes are running.

Usage:
  ros2 launch eos_dataset_player euroc_eos_demo.launch.py \\
    dataset_path:=/path/to/MH_01_easy

Optional:
  replay_rate:=20.0
  rviz:=false    # skip RViz (e.g. run RViz in another terminal)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    eos_slam_share = get_package_share_directory('eos_slam')
    slam_launch = os.path.join(eos_slam_share, 'launch', 'eos_slam_phase123.launch.py')
    rviz_cfg = os.path.join(eos_slam_share, 'rviz', 'eos_slam_phase123.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'dataset_path',
            description='Path to EuRoC dataset root (e.g. .../MH_01_easy)',
        ),
        DeclareLaunchArgument(
            'replay_rate',
            default_value='30.0',
            description='Dataset replay rate (Hz)',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='If true, start RViz with eos_slam_phase123.rviz',
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

        Node(
            package='eos_sensor_bridge',
            executable='sensor_bridge',
            name='eos_sensor_bridge',
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
