from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # === Paths ===
    tb3_gazebo_launch = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'launch',
        'turtlebot3_world.launch.py'
    )

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    localization_launch = os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
    navigation_launch = os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')

    rviz_config = '/home/sanjay/ros2_workspaces/nav_turtlebot/src/slam_pkg/config/nav2_rviz_config.rviz'
    map_file = '/home/sanjay/ros2_workspaces/nav_turtlebot/src/slam_pkg/maps/slam_map.yaml'
    params_file = '/home/sanjay/ros2_workspaces/nav_turtlebot/src/slam_pkg/config/nav2_params.yaml'


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


    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
        launch_arguments={'map': map_file,'params_file': params_file}.items()
    )


    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch)
    )


    return LaunchDescription([
        set_tb3_model,
        gazebo,
        rviz,
        localization,
        navigation
    ])
