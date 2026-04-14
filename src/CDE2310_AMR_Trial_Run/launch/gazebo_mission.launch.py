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
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Force CycloneDDS to use loopback for local Gazebo simulation.
os.environ.pop('CYCLONEDDS_URI', None)
os.environ['ROS_DOMAIN_ID'] = '0'

# Add our custom models (AprilTag textures) to Gazebo's model path
_pkg_share = get_package_share_directory('CDE2310_AMR_Trial_Run')
_models_dir = os.path.join(_pkg_share, 'models')
_existing = os.environ.get('GAZEBO_MODEL_PATH', '')
os.environ['GAZEBO_MODEL_PATH'] = _models_dir + (':' + _existing if _existing else '')


def generate_launch_description():
    # DDS env vars already set at module level above

    # ------------------------------------------------------------------
    # Paths
    # ------------------------------------------------------------------
    pkg_dir = get_package_share_directory('CDE2310_AMR_Trial_Run')
    nav2_yaml = os.path.join(pkg_dir, 'config', 'minimal_nav2.yaml')
    cartographer_config_dir = os.path.join(pkg_dir, 'config')

    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    rviz_config = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz', 'nav2_default_view.rviz')

    tb3_model = os.environ.get('TURTLEBOT3_MODEL', 'waffle_pi')
    model_sdf = os.path.join(
        tb3_gazebo_dir, 'models',
        'turtlebot3_' + tb3_model, 'model.sdf')
    # Default to our custom AMR maze; fall back to turtlebot3_world
    amr_maze = os.path.join(pkg_dir, 'worlds', 'amr_maze.world')
    gazebo_world = amr_maze if os.path.isfile(amr_maze) else os.path.join(
        tb3_gazebo_dir, 'worlds', 'turtlebot3_world.world')

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
    #    (broken out from turtlebot3_world.launch.py so we can delay
    #     spawn_entity — the stock file launches everything concurrently
    #     and Gazebo's factory plugin isn't ready in time on WSL2)
    # ==================================================================
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': gazebo_world}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, 'launch',
                         'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # Delay spawn_entity to give gzserver time to load the factory plugin.
    # The spawner itself waits up to 120 s for the /spawn_entity service.
    spawn_robot = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', tb3_model,
                    '-file', model_sdf,
                    '-x', '0.4',
                    '-y', '0.4',
                    '-z', '0.01',
                    '-timeout', '120',
                ],
                output='screen',
            ),
        ],
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

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
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
                'smoother_server',
                'controller_server',
                'behavior_server',
                'bt_navigator',
            ]},
            {'bond_timeout': 30.0},
        ],
        arguments=['--ros-args', '--log-level', 'INFO'],
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
    # 6. AprilTag detection (camera → tag TF)
    # ==================================================================
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        output='screen',
        remappings=[
            ('image_rect', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ],
        parameters=[
            sim_time_param,
            {'family': '36h11'},
            {'size': 0.16},
        ],
    )

    # ==================================================================
    # 7. Auto-explore (conditional)
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

        # 1. Gazebo  (gzserver first, then client + RSP + delayed spawn)
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_robot,

        # 2. SLAM
        cartographer_node,
        cartographer_occupancy_grid_node,

        # 3. Nav2
        planner_server,
        smoother_server,
        controller_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager_navigation,

        # 4. RViz
        rviz_node,

        # 5. AprilTag detection
        apriltag_node,

        # 6. Mission
        search_server,
        docking_server,
        mission_coordinator,

        # 7. Auto-explore
        find_frontiers,
        score_and_post,
    ])
