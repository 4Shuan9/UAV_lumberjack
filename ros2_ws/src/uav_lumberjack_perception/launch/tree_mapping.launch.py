from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uav_lumberjack_perception',
            executable='tree_semantic_mapper',
            name='tree_semantic_mapper',
            output='screen',
            emulate_tty=True,
            parameters=[PathJoinSubstitution([
                FindPackageShare('uav_lumberjack_perception'),
                'config', 'tree_mapping.yaml'
            ]), {'use_sim_time': True}],
            additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': '{message}'},
        ),
    ])
