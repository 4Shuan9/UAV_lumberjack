import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    project_root = LaunchConfiguration('project_root').perform(context)
    px4_root = LaunchConfiguration('px4_root').perform(context)

    # ============================================================
    # Step13 branch perception paths
    # ============================================================

    step13_root = os.path.join(
        project_root,
        'sim',
        'step13_branch_perception'
    )

    world_file = os.path.join(
        step13_root,
        'worlds',
        'x500_lumberjack_world.sdf'
    )

    model_path = os.path.join(
        step13_root,
        'models'
    )

    bridge_config = os.path.join(
        step13_root,
        'bridge_gazebo_ros2.yaml'
    )

    px4_binary = os.path.join(
        px4_root,
        'build',
        'px4_sitl_default',
        'bin',
        'px4'
    )

    # ============================================================
    # Gazebo resource path
    # ============================================================

    old_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')

    gz_resource_path = model_path

    if old_gz_path:
        gz_resource_path += ':' + old_gz_path

    # ============================================================
    # Cleanup old Gazebo / PX4 processes
    #
    # Important:
    # "[g]z sim" prevents pkill from matching this cleanup command
    # itself.
    # ============================================================

    cleanup = ExecuteProcess(
        cmd=[
            'bash',
            '-c',
            'echo "[LAUNCH] Cleaning old Gazebo / PX4 processes..."; '
            'pkill -f "[g]z sim" 2>/dev/null || true; '
            'pkill -x px4 2>/dev/null || true; '
            'sleep 0.5; '
            'echo "[LAUNCH] Cleanup complete."'
        ],
        output='screen'
    )

    # ============================================================
    # Gazebo
    # ============================================================

    gazebo = ExecuteProcess(
        cmd=[
            'gz',
            'sim',
            '-v',
            '4',
            '-r',
            world_file
        ],
        cwd=step13_root,
        additional_env={
            'GZ_SIM_RESOURCE_PATH': gz_resource_path
        },
        output='screen'
    )

    # ============================================================
    # ROS2 <-> Gazebo bridge
    #
    # Start 1 second after Gazebo starts
    # ============================================================

    bridge = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                parameters=[
                    {
                        'config_file': bridge_config
                    }
                ],
                output='screen'
            )
        ]
    )

    # ============================================================
    # Target branch contact monitor
    #
    # Start after the Gazebo <-> ROS bridge is available.
    # ============================================================

    target_contact_monitor = TimerAction(
        period=1.5,
        actions=[
            Node(
                package='uav_lumberjack_control',
                executable='target_contact_monitor',
                output='screen'
            )
        ]
    )

    # ============================================================
    # Automatic cutting controller
    #
    # Uses target-contact state + actual saw RPM.
    # Publishes ROS /target_branch/detach after 0.8 s effective cut.
    # ============================================================

    auto_cut_controller = TimerAction(
        period=1.7,
        actions=[
            Node(
                package='uav_lumberjack_control',
                executable='auto_cut_controller',
                output='screen'
            )
        ]
    )


    # ============================================================
    # MID360 static TF chain
    #
    # base_link -> mid360_mount_link -> mid360_link -> lidar frame
    # These were manually verified in RViz before being frozen here.
    # ============================================================

    tf_base_to_mid360_mount = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_mid360_mount',
        arguments=[
            '--x', '0.099',
            '--y', '0',
            '--z', '0.010',
            '--roll', '0',
            '--pitch', '0.349066',
            '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'mid360_mount_link'
        ],
        output='screen'
    )

    tf_mid360_mount_to_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_mid360_mount_to_link',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0.033',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '3.141593',
            '--frame-id', 'mid360_mount_link',
            '--child-frame-id', 'mid360_link'
        ],
        output='screen'
    )

    tf_mid360_link_to_sensor = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_mid360_link_to_sensor',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'mid360_link',
            '--child-frame-id',
            'x500_lumberjack/mid360_link/mid360_gpu_lidar'
        ],
        output='screen'
    )

    # ============================================================
    # PX4 SITL
    #
    # Start 3 seconds after Gazebo starts
    # ============================================================

    px4 = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    px4_binary
                ],
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

    # ============================================================
    # Start simulation only AFTER cleanup exits
    #
    # Event handler must be registered before cleanup starts.
    # ============================================================

    start_after_cleanup = RegisterEventHandler(
        OnProcessExit(
            target_action=cleanup,
            on_exit=[
                gazebo,
                bridge,
                target_contact_monitor,
                auto_cut_controller,
                tf_base_to_mid360_mount,
                tf_mid360_mount_to_link,
                tf_mid360_link_to_sensor,
                px4
            ]
        )
    )

    return [
        start_after_cleanup,
        cleanup
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

        OpaqueFunction(
            function=launch_setup
        )
    ])