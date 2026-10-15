"""Independent Stage 2 environment LiDAR preprocessing. No cutting nodes."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('uav_lumberjack_perception'),
        'config', 'environment_filter.yaml'])
    return LaunchDescription([
        Node(
            package='uav_lumberjack_perception',
            executable='environment_preprocessor',
            name='environment_preprocessor',
            output='screen',
            parameters=[config, {'use_sim_time': True}],
        ),
    ])
