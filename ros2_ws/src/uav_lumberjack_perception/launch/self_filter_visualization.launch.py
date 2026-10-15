"""Stage 2 optional RViz debug view of the actual self-filter volumes."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('uav_lumberjack_perception'),
        'config', 'environment_filter.yaml'])
    return LaunchDescription([
        Node(package='uav_lumberjack_perception',
             executable='self_filter_visualizer',
             name='self_filter_visualizer',
             output='screen',
             parameters=[config, {'use_sim_time': True}]),
    ])
