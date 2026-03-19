#!/usr/bin/env python3
"""
Unified launch for CDE2310 AMR — simulation or real hardware.

Simulation (default):
  ros2 launch amr_nav launch.py
  ros2 launch amr_nav launch.py use_rviz:=false use_gui:=true

Real hardware (RPi runs robot.launch.py + camera.launch.py separately):
  ros2 launch amr_nav launch.py use_sim:=false
  ros2 launch amr_nav launch.py use_sim:=false marker_size:=0.08
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    amr_nav_dir = get_package_share_directory('amr_nav')
    slam_params = os.path.join(amr_nav_dir, 'config', 'slam_params.yaml')
    nav_tuning = os.path.join(amr_nav_dir, 'config', 'nav_tuning.yaml')

    use_sim = LaunchConfiguration('use_sim')
    use_rviz = LaunchConfiguration('use_rviz')
    is_sim = PythonExpression(["'", use_sim, "' == 'true'"])

    # ── Arguments ──────────────────────────────────────────
    args = [
        DeclareLaunchArgument('use_sim', default_value='true',
                              description='true = Gazebo, false = real robot'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_gui', default_value='false',
                              description='Show Gazebo GUI (sim only)'),
        DeclareLaunchArgument('x_pose', default_value='-2.0',
                              description='Spawn X (sim only)'),
        DeclareLaunchArgument('y_pose', default_value='-0.5',
                              description='Spawn Y (sim only)'),
        DeclareLaunchArgument('coverage_threshold', default_value='0.95'),
        DeclareLaunchArgument('marker_size', default_value='0.16',
                              description='AprilTag side length in metres'),
    ]

    # ── Gazebo (sim only) ──────────────────────────────────
    gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    world = os.path.join(gazebo_dir, 'worlds', 'turtlebot3_world.world')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items(),
        condition=IfCondition(use_sim))

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(PythonExpression([
            "'", use_sim, "' == 'true' and '",
            LaunchConfiguration('use_gui'), "' == 'true'"
        ])))

    robot_state_pub = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_dir, 'launch',
                         'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(use_sim))

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_dir, 'launch',
                         'spawn_turtlebot3.launch.py')),
        launch_arguments={
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
        }.items(),
        condition=IfCondition(use_sim))

    # ── SIM MODE: delayed SLAM + nav (wait for Gazebo) ─────
    sim_group = GroupAction(
        condition=IfCondition(use_sim),
        actions=[
            TimerAction(period=10.0, actions=[
                Node(
                    package='slam_toolbox',
                    executable='sync_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',
                    parameters=[slam_params, {'use_sim_time': True}]),
            ]),
            TimerAction(period=12.0, actions=[
                Node(
                    package='amr_nav',
                    executable='auto_nav',
                    name='auto_nav',
                    output='screen',
                    parameters=[nav_tuning, {'use_sim_time': True}]),
                Node(
                    package='amr_nav',
                    executable='coverage_monitor',
                    name='coverage_monitor',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'coverage_threshold':
                            LaunchConfiguration('coverage_threshold'),
                    }]),
            ]),
            TimerAction(period=10.0, actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    parameters=[{'use_sim_time': True}],
                    condition=IfCondition(use_rviz)),
            ]),
        ])

    # ── REAL MODE: everything immediate + camera TFs ───────
    real_group = GroupAction(
        condition=UnlessCondition(use_sim),
        actions=[
            Node(
                package='slam_toolbox',
                executable='sync_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[slam_params]),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_to_camera_link',
                arguments=[
                    '--x', '0.07', '--y', '0', '--z', '0.15',
                    '--roll', '0', '--pitch', '0', '--yaw', '0',
                    '--frame-id', 'base_link',
                    '--child-frame-id', 'camera_link',
                ]),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='camera_link_to_optical',
                arguments=[
                    '--x', '0', '--y', '0', '--z', '0',
                    '--roll', '-1.5708', '--pitch', '0',
                    '--yaw', '-1.5708',
                    '--frame-id', 'camera_link',
                    '--child-frame-id', 'camera_optical_frame',
                ]),
            Node(
                package='amr_nav',
                executable='auto_nav',
                name='auto_nav',
                output='screen',
                parameters=[nav_tuning]),
            Node(
                package='amr_perception',
                executable='apriltag_detector',
                name='apriltag_detector',
                output='screen',
                parameters=[{
                    'marker_size': LaunchConfiguration('marker_size'),
                    'detection_rate': 10.0,
                }]),
            Node(
                package='amr_nav',
                executable='coverage_monitor',
                name='coverage_monitor',
                output='screen',
                parameters=[{
                    'coverage_threshold':
                        LaunchConfiguration('coverage_threshold'),
                }]),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                condition=IfCondition(use_rviz)),
        ])

    return LaunchDescription(
        args
        + [gzserver, gzclient, robot_state_pub, spawn_robot]
        + [sim_group, real_group]
    )
