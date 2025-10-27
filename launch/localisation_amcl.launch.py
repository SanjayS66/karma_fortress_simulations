import os
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Path to map and RViz config
    # map_file = '/home/sanjay/ros2_workspaces/nav_turtlebot/src/slam_pkg/maps/slam_map.yaml'
    map_file = '/home/sanjay/Desktop/gazebo_custom_map.world'
    rviz_config = '/home/sanjay/ros2_workspaces/nav_turtlebot/src/slam_pkg/config/localisation_rviz_config.rviz'

    return LaunchDescription([

        # Map server (loads saved map)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file,
                         'use_sim_time': True}]
        ),

        # Lifecycle manager for map_server
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server']}]
        ),

        # AMCL node
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{'use_sim_time': True}],
            respawn_delay=5.0
        ),

        # Lifecycle manager for AMCL
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_amcl',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['amcl']}]
        ),

        # RViz2 with localization config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_node',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}]
        ),

        # Gazebo world (same as in mapping)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('turtlebot3_gazebo'),'/launch','/turtlebot3_world.launch.py'])
        )
    ])
