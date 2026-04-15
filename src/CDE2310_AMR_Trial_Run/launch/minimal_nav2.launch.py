import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Paths
    pkg_dir = get_package_share_directory('CDE2310_AMR_Trial_Run')
    nav2_yaml = os.path.join(pkg_dir, 'config', 'minimal_nav2.yaml')
    slam_yaml = os.path.join(pkg_dir, 'config', 'slam_params.yaml')
    ekf_yaml = os.path.join(pkg_dir, 'config', 'ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    lifecycle_nodes = [
        'planner_server',
        'controller_server',
        'behavior_server',
        'bt_navigator'
    ]

    return LaunchDescription([
        declare_use_sim_time_cmd,

        # --- 1. EKF (odom + IMU fusion) ---
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_yaml, {'use_sim_time': use_sim_time}]
        ),

        # --- 2. SLAM TOOLBOX (async) ---
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_yaml, {'use_sim_time': use_sim_time}]
        ),

        # --- 2. CORE NAV2 SERVERS ---
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_yaml, {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_yaml, {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_yaml, {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_yaml, {'use_sim_time': use_sim_time}]
        ),

        # --- 3. LIFECYCLE MANAGER ---
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': True},
                {'node_names': lifecycle_nodes},
                {'bond_timeout': 8.0}
            ]
        )
    ])
