import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    project_root = LaunchConfiguration('project_root').perform(context)
    px4_root = LaunchConfiguration('px4_root').perform(context)

    step6_root = os.path.join(
        project_root,
        'sim',
        'step6_saw'
    )

    world_file = os.path.join(
        step6_root,
        'worlds',
        'x500_lumberjack_world.sdf'
    )

    model_path = os.path.join(
        step6_root,
        'models'
    )

    bridge_config = os.path.join(
        project_root,
        'sim',
        'step6_saw',
        'bridge_x500_arm_saw.yaml'
    )

    px4_binary = os.path.join(
        px4_root,
        'build',
        'px4_sitl_default',
        'bin',
        'px4'
    )

    old_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = model_path
    if old_gz_path:
        gz_resource_path += ':' + old_gz_path

    gazebo = ExecuteProcess(
        cmd=[
            'gz',
            'sim',
            '-v',
            '4',
            '-r',
            world_file
        ],
        cwd=step6_root,
        additional_env={
            'GZ_SIM_RESOURCE_PATH': gz_resource_path
        },
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
                    'PX4_GZ_MODEL_NAME': 'x500_lumberjack',
                    'GZ_SIM_RESOURCE_PATH': gz_resource_path
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
