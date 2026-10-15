from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    common_parameters = [
        {
            'use_sim_time': True
        }
    ]

    # Remove the duplicate "[INFO] [time] [node]:" portion generated
    # by rcutils. ros2 launch will still prepend its short process tag,
    # e.g. [target-3]. The "-3" is launch's automatic process index.
    clean_env = {
        'RCUTILS_CONSOLE_OUTPUT_FORMAT': '{message}'
    }

    environment_config = PathJoinSubstitution([
        FindPackageShare('uav_lumberjack_perception'),
        'config', 'environment_filter.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('environment', default_value='true',
                              description='Enable Stage 2 LiDAR self/ground filtering'),
        DeclareLaunchArgument('self_filter_viz', default_value='true',
                              description='Publish RViz2 self-filter MarkerArray'),
        Node(
            package='uav_lumberjack_perception',
            executable='tfsrc',
            name='tfsrc',
            output='screen',
            emulate_tty=True,
            parameters=common_parameters,
            additional_env=clean_env
        ),

        Node(
            package='uav_lumberjack_perception',
            executable='mask',
            name='mask',
            output='screen',
            emulate_tty=True,
            parameters=common_parameters,
            additional_env=clean_env,
            arguments=[
                '--ros-args',
                '--log-level',
                'warn'
            ]
        ),

        Node(
            package='uav_lumberjack_perception',
            executable='target',
            name='target',
            output='screen',
            emulate_tty=True,
            parameters=common_parameters,
            additional_env=clean_env
        ),

        Node(
            package='uav_lumberjack_perception',
            executable='worldtf',
            name='worldtf',
            output='screen',
            emulate_tty=True,
            parameters=common_parameters,
            additional_env=clean_env
        ),

        # Single-frame geometry remains active for RViz / A-B
        # comparison, but routine INFO is hidden.
        Node(
            package='uav_lumberjack_perception',
            executable='single',
            name='single',
            output='screen',
            emulate_tty=True,
            parameters=common_parameters,
            additional_env=clean_env,
            arguments=[
                '--ros-args',
                '--log-level',
                'warn'
            ]
        ),

        Node(
            package='uav_lumberjack_perception',
            executable='fusion',
            name='fusion',
            output='screen',
            emulate_tty=True,
            parameters=common_parameters,
            additional_env=clean_env
        ),

        Node(
            package='uav_lumberjack_perception',
            executable='model',
            name='model',
            output='screen',
            emulate_tty=True,
            parameters=common_parameters,
            additional_env=clean_env
        ),

        # Stage 2: shared for validation_world and agriculture_world.
        # Both bridges provide /mid360/points, joint_states, and Gazebo odom.
        Node(
            package='uav_lumberjack_perception',
            executable='environment_preprocessor',
            name='environment_preprocessor',
            output='screen',
            emulate_tty=True,
            parameters=[environment_config, {'use_sim_time': True}],
            additional_env=clean_env,
            condition=IfCondition(LaunchConfiguration('environment')),
        ),
        Node(
            package='uav_lumberjack_perception',
            executable='self_filter_visualizer',
            name='self_filter_visualizer',
            output='screen',
            emulate_tty=True,
            parameters=common_parameters,
            additional_env=clean_env,
            condition=IfCondition(LaunchConfiguration('self_filter_viz')),
        ),
    ])
