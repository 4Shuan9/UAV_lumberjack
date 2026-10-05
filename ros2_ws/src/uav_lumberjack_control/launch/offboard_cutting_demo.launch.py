import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('uav_lumberjack_control')
    default_params = os.path.join(share_dir, 'config', 'offboard_cutting_demo.yaml')

    params_file = LaunchConfiguration('params_file')
    start_arm = LaunchConfiguration('start_arm')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Offboard cutting demo parameter YAML'
        ),
        DeclareLaunchArgument(
            'start_arm',
            default_value='true',
            description='Start arm_controller together with the cutting demo node'
        ),

        # Keep simulation and perception launches separate/frozen.
        # This launch contains only task-level control nodes.
        Node(
            package='uav_lumberjack_control',
            executable='arm_controller',
            name='arm_controller',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(start_arm),
        ),

        Node(
            package='uav_lumberjack_control',
            executable='offboard_cutting_demo',
            name='offboard_cutting_demo',
            output='screen',
            emulate_tty=True,
            parameters=[params_file],
        ),
    ])
