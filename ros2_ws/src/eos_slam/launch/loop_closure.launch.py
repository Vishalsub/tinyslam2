from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('eos_slam')
    config  = PathJoinSubstitution([pkg_dir, 'config', 'default_params.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=config),

        Node(
            package='eos_slam',
            executable='loop_closure_node',
            name='eos_loop_closure',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
        ),
    ])
