import os
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='mapping_node',
        output='screen',
        parameters=['/home/sanjay/ros2_workspaces/rcup_migration/src/rcup_garden/config/mapper_params_online_async.yaml',{'use_sim_time':True}]
    )

    rviz2_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_node',
            output='screen',
            arguments=['-d','/home/sanjay/ros2_workspaces/nav_turtlebot/src/slam_pkg/config/slam_rviz_config.rviz'],
            parameters=[{'use_sim_time': True}]
        )
    
    delayed_slam = TimerAction(
        period=35.0,
        actions=[slam_node]
    )

    delayed_rviz = TimerAction(
        period=35.0,
        actions=[rviz2_node]
    )

    return LaunchDescription([
        delayed_slam,
        delayed_rviz,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('rcup_garden'),'/launch','/gazebo_diff.launch.py']))
    ])