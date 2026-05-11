from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('eos_slam')
    config_file = PathJoinSubstitution([pkg_dir, 'config', 'default_params.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=config_file,
                              description='Path to parameter file'),

        Node(
            package='eos_slam',
            executable='mapping_node',
            name='eos_mapper',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
        ),
    ])
