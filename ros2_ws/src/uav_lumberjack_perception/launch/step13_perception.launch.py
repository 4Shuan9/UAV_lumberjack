from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    common_parameters = [
        {
            'use_sim_time': True
        }
    ]

    # ============================================================
    # world -> base_link dynamic TF
    # ============================================================

    odom_to_tf = Node(
        package='uav_lumberjack_perception',
        executable='odom_to_tf',
        name='odom_to_tf',
        output='screen',
        parameters=common_parameters
    )

    # ============================================================
    # RGB red target segmentation
    # ============================================================

    red_branch_detector = Node(
        package='uav_lumberjack_perception',
        executable='red_branch_detector',
        name='red_branch_detector',
        output='screen',
        parameters=common_parameters
    )

    # ============================================================
    # RGB Mask + LiDAR -> target branch cloud @ base_link
    # ============================================================

    target_branch_cloud = Node(
        package='uav_lumberjack_perception',
        executable='target_branch_cloud',
        name='target_branch_cloud',
        output='screen',
        parameters=common_parameters
    )

    # ============================================================
    # target cloud: base_link -> world
    # ============================================================

    target_cloud_world = Node(
        package='uav_lumberjack_perception',
        executable='target_cloud_world',
        name='target_cloud_world',
        output='screen',
        parameters=common_parameters
    )

    # ============================================================
    # Single-frame PCA / geometry estimation
    # ============================================================

    branch_pca = Node(
        package='uav_lumberjack_perception',
        executable='branch_pca',
        name='branch_pca_node',
        output='screen',
        parameters=common_parameters
    )

    return LaunchDescription([
        odom_to_tf,
        red_branch_detector,
        target_branch_cloud,
        target_cloud_world,
        branch_pca,
    ])
