from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # === Paths ===
    tb3_gazebo_launch = os.path.join(
        get_package_share_directory('rcup_garden'),
        'launch',
        'gazebo_diff.launch.py'
    )

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    localization_launch = os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
    navigation_launch = os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')

    rviz_config = '/home/sanjay/ros2_workspaces/rcup_migration/src/rcup_garden/config/nav2_rviz_config.rviz'
    map_file = '/home/sanjay/ros2_workspaces/rcup_migration/src/rcup_garden/maps/slam_map.yaml'
    params_file = '/home/sanjay/ros2_workspaces/rcup_migration/src/rcup_garden/config/nav2_params.yaml'


    for f in [rviz_config, map_file, tb3_gazebo_launch, localization_launch, navigation_launch]:
        if not os.path.exists(f):
            raise FileNotFoundError(f"Missing required file: {f}")


    set_tb3_model = SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value=os.environ.get('TURTLEBOT3_MODEL', 'burger'))


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb3_gazebo_launch)
    )


    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # Relay node to bridge /cmd_vel_nav (from Nav2) to /diff_cont/cmd_vel_unstamped
    # This ensures Nav2 velocity commands reach the diff_drive_controller
    cmd_vel_nav_relay = Node(
        package='rcup_garden',
        executable='cmd_vel_nav_relay',
        name='cmd_vel_nav_relay',
        output='screen'
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
        launch_arguments={'map': map_file,'params_file': params_file}.items()
    )

    # Delay localization by 15 seconds to allow Gazebo to fully initialize
    # and start publishing /diff_cont/odom before EKF tries to fuse it
    delayed_localization = TimerAction(
        period=15.0,
        actions=[localization]
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch),
        launch_arguments={'params_file': params_file}.items()
    )

    # Delay navigation by 20 seconds (gives EKF time to start and publish transforms)
    delayed_navigation = TimerAction(
        period=20.0,
        actions=[navigation]
    )


    return LaunchDescription([
        set_tb3_model,
        gazebo,
        rviz,
        cmd_vel_nav_relay,
        delayed_localization,
        delayed_navigation
    ])
