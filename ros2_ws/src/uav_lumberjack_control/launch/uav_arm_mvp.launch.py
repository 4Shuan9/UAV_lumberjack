import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    project_root = LaunchConfiguration('project_root').perform(context)
    px4_root = LaunchConfiguration('px4_root').perform(context)

    world_file = os.path.join(
        project_root,
        'sim',
        'step5_px4_flight',
        'worlds',
        'x500_lumberjack_world.sdf'
    )

    bridge_config = os.path.join(
        project_root,
        'sim',
        'step4_x500_arm',
        'bridge_x500_arm.yaml'
    )

    px4_binary = os.path.join(
        px4_root,
        'build',
        'px4_sitl_default',
        'bin',
        'px4'
    )

    gazebo = ExecuteProcess(
        cmd=[
            'gz',
            'sim',
            '-v',
            '4',
            '-r',
            world_file
        ],
        output='screen'
    )

    bridge = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                parameters=[
                    {'config_file': bridge_config}
                ],
                output='screen'
            )
        ]
    )

    px4 = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[px4_binary],
                cwd=px4_root,
                additional_env={
                    'PX4_GZ_STANDALONE': '1',
                    'PX4_SYS_AUTOSTART': '4001',
                    'PX4_GZ_MODEL_NAME': 'x500_lumberjack'
                },
                output='screen'
            )
        ]
    )

    return [
        gazebo,
        bridge,
        px4
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'project_root',
            default_value=os.path.expanduser('~/UAV_lumberjack'),
            description='UAV_lumberjack project root'
        ),

        DeclareLaunchArgument(
            'px4_root',
            default_value=os.path.expanduser('~/PX4-Autopilot'),
            description='PX4-Autopilot root'
        ),

        OpaqueFunction(function=launch_setup)
    ])