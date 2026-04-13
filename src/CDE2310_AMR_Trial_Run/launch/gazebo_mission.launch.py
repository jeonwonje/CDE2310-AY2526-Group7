#!/usr/bin/env python3
"""
gazebo_mission.launch.py — Full mission stack in Gazebo simulation.

Launches:
  1. Gazebo with TurtleBot3 world
  2. TurtleBot3 robot state publisher
  3. Cartographer SLAM
  4. Nav2 stack
  5. RViz
  6. Mission nodes (search, docking, coordinator)
  7. Auto-explore (frontier finding + scoring)

Usage:
  export TURTLEBOT3_MODEL=waffle_pi
  ros2 launch CDE2310_AMR_Trial_Run gazebo_mission.launch.py

  # With a different world:
  ros2 launch CDE2310_AMR_Trial_Run gazebo_mission.launch.py \
      world:=turtlebot3_house

  # Mission nodes only (skip mission stack for nav-only testing):
  ros2 launch CDE2310_AMR_Trial_Run gazebo_mission.launch.py \
      launch_mission:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ------------------------------------------------------------------
    # Paths
    # ------------------------------------------------------------------
    pkg_dir = get_package_share_directory('CDE2310_AMR_Trial_Run')
    nav2_yaml = os.path.join(pkg_dir, 'config', 'minimal_nav2.yaml')
    cartographer_config_dir = os.path.join(pkg_dir, 'config')

    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    rviz_config = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz', 'nav2_default_view.rviz')

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    launch_mission = LaunchConfiguration('launch_mission')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation clock')

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='turtlebot3_world',
        description='Gazebo world name (without .launch.py extension)')

    declare_launch_mission = DeclareLaunchArgument(
        'launch_mission',
        default_value='true',
        description='Launch mission and exploration nodes')

    sim_time_param = {'use_sim_time': use_sim_time}

    # ==================================================================
    # 1. GAZEBO + TURTLEBOT3 SPAWN
    # ==================================================================
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, 'launch',
                         'turtlebot3_world.launch.py')),
    )

    # ==================================================================
    # 2. SLAM — Cartographer
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
    # 3. Nav2 stack
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
    # 4. RViz
    # ==================================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config, '--ros-args', '--log-level', 'WARN'],
    )

    # ==================================================================
    # 5. Mission nodes (conditional)
    # ==================================================================
    search_server = Node(
        package='CDE2310_AMR_Trial_Run',
        executable='search_server',
        name='search_server',
        output='screen',
        parameters=[sim_time_param],
        condition=IfCondition(launch_mission),
    )

    docking_server = Node(
        package='CDE2310_AMR_Trial_Run',
        executable='docking_server',
        name='docking_server',
        output='screen',
        parameters=[sim_time_param],
        condition=IfCondition(launch_mission),
    )

    mission_coordinator = Node(
        package='CDE2310_AMR_Trial_Run',
        executable='mission_coordinator',
        name='mission_coordinator',
        output='screen',
        parameters=[sim_time_param],
        condition=IfCondition(launch_mission),
    )

    # ==================================================================
    # 6. Auto-explore (conditional)
    # ==================================================================
    find_frontiers = Node(
        package='auto_explore_v2',
        executable='find_frontiers',
        name='auto_explore',
        output='screen',
        parameters=[sim_time_param],
        condition=IfCondition(launch_mission),
    )

    score_and_post = Node(
        package='auto_explore_v2',
        executable='score_and_post',
        name='score_and_post',
        output='screen',
        parameters=[sim_time_param],
        condition=IfCondition(launch_mission),
    )

    # ==================================================================
    # Assemble
    # ==================================================================
    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        declare_launch_mission,

        # 1. Gazebo
        gazebo_launch,

        # 2. SLAM
        cartographer_node,
        cartographer_occupancy_grid_node,

        # 3. Nav2
        planner_server,
        controller_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager_navigation,

        # 4. RViz
        rviz_node,

        # 5. Mission
        search_server,
        docking_server,
        mission_coordinator,

        # 6. Auto-explore
        find_frontiers,
        score_and_post,
    ])
