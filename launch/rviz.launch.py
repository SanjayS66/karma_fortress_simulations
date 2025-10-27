from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command
import os
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share_dir = get_package_share_directory('rcup_garden')

    # Use the xacro file and call xacro at runtime so the installed package
    # (which contains the .xacro file) will work without needing a preprocessed .urdf
    xacro_file = os.path.join(share_dir, 'urdf', 'ArmPlate.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    rviz_config_file = os.path.join(share_dir, 'config', 'im_so_doneee.rviz')

    # gui_arg = DeclareLaunchArgument(
    #     name='gui',
    #     default_value='True'
    # )

    # show_gui = LaunchConfiguration('gui')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}
        ]
    )

    joint_state_publisher_node = Node(
        # condition=UnlessCondition(show_gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # joint_state_publisher_gui_node = Node(
    #     # condition=IfCondition(show_gui),
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui'
    # )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{"use_sim_time":True}],
    )

    return LaunchDescription([
        # gui_arg,
        
        joint_state_publisher_node,
        robot_state_publisher_node,
        # joint_state_publisher_gui_node,
        rviz_node
    ])
