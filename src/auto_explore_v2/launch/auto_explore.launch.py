import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Define the launch configuration variable
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 2. Declare the argument so it shows up in help/CLI
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    package_name = 'auto_explore_v2' # Change this to your actual package name

    # Node 1: Find Frontiers
    find_frontiers_node = Node(
        package=package_name,
        executable='find_frontiers',
        name='auto_explore',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Node 2: Score and Post
    score_and_post_node = Node(
        package=package_name,
        executable='score_and_post',
        name='score_and_post',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time_argument,
        find_frontiers_node,
        score_and_post_node
    ])
