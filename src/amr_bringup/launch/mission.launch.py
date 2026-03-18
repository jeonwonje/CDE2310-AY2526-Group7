#!/usr/bin/env python3
"""
Unified launch file for the CDE2310 warehouse AMR mission.

Launches on the WSL2 side:
  1. SLAM Toolbox (online_sync mode)
  2. Nav2 stack (with corrected params)
  3. frontier_detector node
  4. apriltag_detector node
  5. mission_controller node (master BT)
  6. Static TF for camera frame
  7. RViz2

RPi side (launch separately):
  ros2 launch turtlebot3_bringup robot.launch.py
  ros2 launch turtlebot3_bringup camera.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Package paths ────────────────────────────────────
    amr_navigation_dir = get_package_share_directory('amr_navigation')
    nav2_params = os.path.join(amr_navigation_dir, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(amr_navigation_dir, 'config', 'slam_params.yaml')

    # ── Launch arguments ─────────────────────────────────
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2')

    marker_size_arg = DeclareLaunchArgument(
        'marker_size', default_value='0.16',
        description='AprilTag marker side length in metres')

    lift_api_url_arg = DeclareLaunchArgument(
        'lift_api_url', default_value='http://localhost:8080/lift',
        description='Station C lift API base URL')

    # ── SLAM Toolbox ─────────────────────────────────────
    slam_node = Node(
        package='slam_toolbox',
        executable='online_sync_launch.py' if False else 'sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params],
    )

    # ── Nav2 Bringup ─────────────────────────────────────
    try:
        nav2_bringup_dir = get_package_share_directory('nav2_bringup')
        nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': nav2_params,
            }.items(),
        )
    except Exception:
        # Fallback: launch Nav2 nodes individually
        nav2_launch = GroupAction([
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                parameters=[nav2_params],
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                output='screen',
                parameters=[nav2_params],
            ),
            Node(
                package='nav2_recoveries',
                executable='recoveries_server',
                output='screen',
                parameters=[nav2_params],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                output='screen',
                parameters=[nav2_params],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                output='screen',
                parameters=[nav2_params],
            ),
        ])

    # ── Static TF: base_link → camera_link ───────────────
    camera_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_link',
        arguments=[
            '--x', '0.07', '--y', '0', '--z', '0.15',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link',
        ],
    )

    # camera_link → camera_optical_frame
    # Standard optical frame convention: z-forward, x-right, y-down
    camera_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_to_optical',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708',
            '--frame-id', 'camera_link',
            '--child-frame-id', 'camera_optical_frame',
        ],
    )

    # ── Frontier Detector ────────────────────────────────
    frontier_detector = Node(
        package='amr_navigation',
        executable='frontier_detector',
        name='frontier_detector',
        output='screen',
    )

    # ── AprilTag Detector ────────────────────────────────
    apriltag_detector = Node(
        package='amr_perception',
        executable='apriltag_detector',
        name='apriltag_detector',
        output='screen',
        parameters=[{
            'marker_size': LaunchConfiguration('marker_size'),
            'detection_rate': 10.0,
        }],
    )

    # ── Mission Controller ───────────────────────────────
    mission_controller = Node(
        package='amr_bringup',
        executable='mission_controller',
        name='mission_controller',
        output='screen',
        parameters=[{
            'lift_api_url': LaunchConfiguration('lift_api_url'),
        }],
    )

    # ── RViz2 ────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    # ── Build launch description ─────────────────────────
    return LaunchDescription([
        use_rviz_arg,
        marker_size_arg,
        lift_api_url_arg,
        slam_node,
        nav2_launch,
        camera_link_tf,
        camera_optical_tf,
        frontier_detector,
        apriltag_detector,
        mission_controller,
        rviz_node,
    ])
