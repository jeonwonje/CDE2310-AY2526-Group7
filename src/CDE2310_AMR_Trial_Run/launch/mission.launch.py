import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Create the Launch Configuration variable
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 2. Declare the argument so it can be passed via command line
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # 3. Create the parameter dictionary to pass to all nodes
    sim_time_param = {'use_sim_time': use_sim_time}

    # 4. Define the Nodes
    search_node = Node(
        package='CDE2310_AMR_Trial_Run',
        executable='search_server',
        name='search_server',
        output='screen',
        parameters=[sim_time_param]
    )

    docking_node = Node(
        package='CDE2310_AMR_Trial_Run',
        executable='docking_server',
        name='docking_server',
        output='screen',
        parameters=[sim_time_param]
    )

    coordinator_node = Node(
        package='CDE2310_AMR_Trial_Run',
        executable='mission_coordinator',
        name='mission_coordinator',
        output='screen',
        parameters=[sim_time_param]
    )
    
    delivery_node = Node(
        package='CDE2310_AMR_Trial_Run',
        executable='delivery_server',
        name='delivery_server',
        output='screen',
        parameters=[sim_time_param]
    )

    return LaunchDescription([
        # MUST add the declaration action before the nodes
        declare_use_sim_time_cmd,
        search_node,
        docking_node,
        delivery_node,
        coordinator_node
    ])