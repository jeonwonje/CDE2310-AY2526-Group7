#!/usr/bin/env python3
"""
Gazebo simulation launch file for wavefront frontier exploration benchmarking.

Launches:
  1. gzserver (headless — no gzclient, which kills WSL2 with software rendering)
  2. robot_state_publisher + spawn TurtleBot3
  3. SLAM Toolbox (online_sync mode, sim time)  [delayed 10s]
  4. Nav2 stack (sim time)                       [delayed 10s]
  5. frontier_detector                           [delayed 15s]
  6. exploration_manager                         [delayed 15s]
  7. coverage_monitor (publishes /map_closed)    [delayed 15s]
  8. RViz2 (optional)                            [delayed 10s]

Overrides CYCLONEDDS_URI to a localhost-only config so that DDS discovery
works between gzserver's internal ROS nodes and the launched nodes (the
default cyclonedds.xml is tuned for WSL2 ↔ RPi communication).

Usage:
  ros2 launch amr_bringup sim_exploration.launch.py
  ros2 launch amr_bringup sim_exploration.launch.py use_rviz:=false coverage_threshold:=0.90
  ros2 launch amr_bringup sim_exploration.launch.py use_gui:=true   # enable Gazebo GUI (needs GPU)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Package paths ────────────────────────────────────
    amr_navigation_dir = get_package_share_directory('amr_navigation')
    nav2_params_sim = os.path.join(amr_navigation_dir, 'config', 'nav2_params_sim.yaml')
    slam_params_sim = os.path.join(amr_navigation_dir, 'config', 'slam_params_sim.yaml')
    gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Note: cyclonedds_sim.xml is available if needed, but the user's
    # default ~/cyclonedds.xml works for same-machine DDS discovery.
    # The key fix is running headless (no gzclient) to avoid CPU starvation.

    # ── Launch arguments ─────────────────────────────────
    x_pose_arg = DeclareLaunchArgument(
        'x_pose', default_value='-2.0',
        description='Initial X position in Gazebo world')

    y_pose_arg = DeclareLaunchArgument(
        'y_pose', default_value='-0.5',
        description='Initial Y position in Gazebo world')

    coverage_threshold_arg = DeclareLaunchArgument(
        'coverage_threshold', default_value='0.95',
        description='Map coverage fraction to declare exploration complete')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2')

    use_gui_arg = DeclareLaunchArgument(
        'use_gui', default_value='false',
        description='Launch Gazebo GUI (gzclient) — disabled by default on WSL2')

    # ── 1. Gazebo server (headless by default) ───────────
    world = os.path.join(gazebo_dir, 'worlds', 'turtlebot3_world.world')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_gui')),
    )

    # ── 2. Robot state publisher + spawn ──────────────────
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_dir, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_dir, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
        }.items(),
    )

    # ── 3. SLAM Toolbox (delayed — needs Gazebo TF) ─────
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_sim, {'use_sim_time': True}],
    )

    # ── 4. Nav2 stack (delayed — needs Gazebo TF) ───────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_sim,
        }.items(),
    )

    # ── 5. Frontier detector ─────────────────────────────
    frontier_detector = Node(
        package='amr_navigation',
        executable='frontier_detector',
        name='frontier_detector',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ── 6. Exploration manager ───────────────────────────
    exploration_manager = Node(
        package='amr_navigation',
        executable='exploration_manager',
        name='exploration_manager',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ── 7. Coverage monitor ──────────────────────────────
    coverage_monitor = Node(
        package='amr_navigation',
        executable='coverage_monitor',
        name='coverage_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'coverage_threshold': LaunchConfiguration('coverage_threshold'),
        }],
    )

    # ── 8. RViz2 ─────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    # ── Build launch description ─────────────────────────
    # gzserver starts immediately (headless). SLAM + Nav2 delayed 10s
    # to let physics + robot spawn complete. Exploration delayed 15s
    # to let Nav2 lifecycle come up.
    return LaunchDescription([
        # Arguments
        x_pose_arg,
        y_pose_arg,
        coverage_threshold_arg,
        use_rviz_arg,
        use_gui_arg,
        # Gazebo headless (immediate)
        gzserver,
        gzclient,  # only if use_gui:=true
        robot_state_publisher,
        spawn_turtlebot,
        # SLAM + Nav2 (delayed 10s for Gazebo init)
        TimerAction(period=10.0, actions=[slam_node, nav2_launch]),
        # Exploration + coverage (delayed 15s for Nav2 lifecycle)
        TimerAction(period=15.0, actions=[
            frontier_detector,
            exploration_manager,
            coverage_monitor,
        ]),
        # RViz (delayed 10s)
        TimerAction(period=10.0, actions=[rviz_node]),
    ])
