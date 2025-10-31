import os
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([


        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='mapping_node',
            output='screen',
            parameters=['/home/sanjay/ros2_workspaces/rcup_migration/src/rcup_garden/config/mapper_params_online_async.yaml',{'use_sim_time':True}]
        ),


        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_node',
            output='screen',
            arguments=['-d','/home/sanjay/ros2_workspaces/nav_turtlebot/src/slam_pkg/config/slam_rviz_config.rviz'],
            parameters=[{'use_sim_time': True}]
        ),

            IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('rcup_garden'),'/launch','/gazebo_diff.launch.py']))
    ])