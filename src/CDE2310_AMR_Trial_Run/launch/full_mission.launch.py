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
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ---------------------------------------------------------------------------
# DDS config split — only nodes that MUST talk to the RPi get the cross-
# network config.  Everything else stays localhost-only so DDS discovery
# and data traffic don't saturate the 2.4 GHz WiFi hotspot link.
#
#   Cross-network (cyclonedds.xml):  SLAM, EKF, controller, mission nodes
#   Localhost-only (cyclonedds_local.xml): planner, behavior, bt_nav,
#                                          lifecycle mgr, rviz, explore
# ---------------------------------------------------------------------------
DDS_CROSS = {'CYCLONEDDS_URI': 'file:///home/jeon-ros2-humble-gazebo-wsl/cyclonedds.xml'}
DDS_LOCAL = {'CYCLONEDDS_URI': 'file:///home/jeon-ros2-humble-gazebo-wsl/cyclonedds_local.xml'}


def generate_launch_description():
    # ------------------------------------------------------------------
    # Paths
    # ------------------------------------------------------------------
    pkg_dir = get_package_share_directory('CDE2310_AMR_Trial_Run')
    nav2_yaml = os.path.join(pkg_dir, 'config', 'minimal_nav2.yaml')
    slam_yaml = os.path.join(pkg_dir, 'config', 'slam_params.yaml')
    ekf_yaml = os.path.join(pkg_dir, 'config', 'ekf.yaml')
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
    # 1. EKF — fuse wheel odom + IMU for better heading in tight corridors
    # ==================================================================
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_yaml, sim_time_param],
        additional_env=DDS_CROSS,
    )

    # ==================================================================
    # 2. SLAM — SLAM Toolbox on Remote PC
    # ==================================================================
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_yaml, sim_time_param],
        additional_env=DDS_CROSS,  # needs /scan from RPi, publishes /map
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
        additional_env=DDS_LOCAL,
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_yaml, sim_time_param],
        arguments=['--ros-args', '--log-level', 'WARN'],
        additional_env=DDS_CROSS,  # needs /scan from RPi, publishes /cmd_vel to RPi
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_yaml, sim_time_param],
        arguments=['--ros-args', '--log-level', 'WARN'],
        additional_env=DDS_LOCAL,
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_yaml, sim_time_param],
        arguments=['--ros-args', '--log-level', 'WARN'],
        additional_env=DDS_LOCAL,
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
            {'bond_timeout': 30.0},
        ],
        arguments=['--ros-args', '--log-level', 'INFO'],
        additional_env=DDS_LOCAL,
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
        additional_env=DDS_LOCAL,
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
        additional_env=DDS_LOCAL,
    )

    docking_server = Node(
        package='CDE2310_AMR_Trial_Run',
        executable='docking_server',
        name='docking_server',
        output='screen',
        parameters=[sim_time_param],
        additional_env=DDS_CROSS,  # needs RPi dock pose topics + delivery service
    )

    # NOTE: delivery_server runs on the RPi (needs local access to /fire_ball service)

    mission_coordinator = Node(
        package='CDE2310_AMR_Trial_Run',
        executable='mission_coordinator',
        name='mission_coordinator',
        output='screen',
        parameters=[sim_time_param],
        additional_env=DDS_CROSS,  # calls services on RPi
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
        additional_env=DDS_LOCAL,
    )

    score_and_post = Node(
        package='auto_explore_v2',
        executable='score_and_post',
        name='score_and_post',
        output='screen',
        parameters=[sim_time_param],
        additional_env=DDS_LOCAL,
    )

    # ==================================================================
    # Assemble
    # ==================================================================
    return LaunchDescription([
        declare_use_sim_time,

        # 1. EKF (odom + IMU fusion)
        ekf_node,

        # 2. SLAM
        slam_toolbox_node,

        # 3. Nav2
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
