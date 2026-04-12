#!/usr/bin/env python3
"""
full_mission.launch.py — Single launch file for the complete remote PC mission.

Replaces these 4 terminal commands:
  1. ros2 launch CDE2310_AMR_Trial_Run minimal_nav2.launch.py use_sim_time:=false
  2. ros2 run rviz2 rviz2 -d .../nav2_default_view.rviz
  3. ros2 launch CDE2310_AMR_Trial_Run mission.launch.py use_sim_time:=false
  4. ros2 launch auto_explore_v2 auto_explore.launch.py use_sim_time:=false

Usage:
  ros2 launch CDE2310_AMR_Trial_Run full_mission.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ------------------------------------------------------------------
    # Paths
    # ------------------------------------------------------------------
    pkg_dir = get_package_share_directory('CDE2310_AMR_Trial_Run')
    nav2_yaml = os.path.join(pkg_dir, 'config', 'minimal_nav2.yaml')
    cartographer_config_dir = os.path.join(pkg_dir, 'config')
    rviz_config = os.path.join(
        get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    sim_time_param = {'use_sim_time': use_sim_time}

    # ==================================================================
    # 1. SLAM — Cartographer
    # ==================================================================
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[sim_time_param],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', 'cartographer.lua',
            '--ros-args', '--log-level', 'WARN',
        ],
    )

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[
            sim_time_param,
            {'resolution': 0.05},
            {'publish_period_sec': 1.0},
        ],
        arguments=['--ros-args', '--log-level', 'WARN'],
    )

    # ==================================================================
    # 2. Nav2 stack
    # ==================================================================
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_yaml, sim_time_param],
        arguments=['--ros-args', '--log-level', 'WARN'],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_yaml, sim_time_param],
        arguments=['--ros-args', '--log-level', 'WARN'],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_yaml, sim_time_param],
        arguments=['--ros-args', '--log-level', 'WARN'],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_yaml, sim_time_param],
        arguments=['--ros-args', '--log-level', 'WARN'],
    )

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            sim_time_param,
            {'autostart': True},
            {'node_names': [
                'planner_server',
                'controller_server',
                'behavior_server',
                'bt_navigator',
            ]},
            {'bond_timeout': 8.0},
        ],
        arguments=['--ros-args', '--log-level', 'WARN'],
    )

    # ==================================================================
    # 3. RViz
    # ==================================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config, '--ros-args', '--log-level', 'WARN'],
    )

    # ==================================================================
    # 4. Mission nodes (search, docking, delivery, coordinator)
    # ==================================================================
    search_server = Node(
        package='CDE2310_AMR_Trial_Run',
        executable='search_server',
        name='search_server',
        output='screen',
        parameters=[sim_time_param],
    )

    docking_server = Node(
        package='CDE2310_AMR_Trial_Run',
        executable='docking_server',
        name='docking_server',
        output='screen',
        parameters=[sim_time_param],
    )

    # NOTE: delivery_server runs on the RPi (needs local access to /fire_ball service)

    mission_coordinator = Node(
        package='CDE2310_AMR_Trial_Run',
        executable='mission_coordinator',
        name='mission_coordinator',
        output='screen',
        parameters=[sim_time_param],
    )

    # ==================================================================
    # 5. Auto-explore (frontier finding + scoring)
    # ==================================================================
    find_frontiers = Node(
        package='auto_explore_v2',
        executable='find_frontiers',
        name='auto_explore',
        output='screen',
        parameters=[sim_time_param],
    )

    score_and_post = Node(
        package='auto_explore_v2',
        executable='score_and_post',
        name='score_and_post',
        output='screen',
        parameters=[sim_time_param],
    )

    # ==================================================================
    # Assemble
    # ==================================================================
    return LaunchDescription([
        declare_use_sim_time,

        # 1. SLAM
        cartographer_node,
        cartographer_occupancy_grid_node,

        # 2. Nav2
        planner_server,
        controller_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager_navigation,

        # 3. RViz
        rviz_node,

        # 4. Mission
        search_server,
        docking_server,
        mission_coordinator,

        # 5. Auto-explore
        find_frontiers,
        score_and_post,
    ])
