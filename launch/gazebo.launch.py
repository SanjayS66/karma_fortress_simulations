import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler,ExecuteProcess,TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get the share directory of your package
    package_name = 'rcup_garden'
    pkg_share = get_package_share_directory(package_name)


    # Prefer processing the xacro at runtime (installed package contains the .xacro)
    xacro_file = os.path.join(pkg_share, 'urdf', 'ArmPlate.urdf.xacro')
    # Path to the controller config file
    # controllers_yaml_path = os.path.join(pkg_share, 'config', 'mecanum_controllers.yaml')
    controllers_yaml_path = os.path.join(pkg_share, 'config', 'diff_drive_controllers.yaml')
    # --- Robot Description in XML---
    robot_description_xml = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # --- Launch Arguments ---
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')



    declare_x_pose_arg = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Initial X position of the robot'
    )
    declare_y_pose_arg = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Initial Y position of the robot'
    )

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    #world launcher arguement
    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty_world.world'
        )    
    
    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='world to load'
    )

    on_exit_shutdown = LaunchConfiguration('on_exit_shutdown')
    
    declare_on_exit_shutdown_arg = DeclareLaunchArgument(
        'on_exit_shutdown',
        default_value='true',
        description='Shutdown launch when Gazebo exits'
    )

    # Gazebo server (physics engine, headless)
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [' -r ',' -s ', ' -v4 ', world],
            'use_sim_time': 'true',
            'on_exit_shutdown': on_exit_shutdown
        }.items()
    )

    # Gazebo client (GUI)
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-g',
            'on_exit_shutdown': on_exit_shutdown
        }.items()
    )      

    # --- Spawning Robot ---
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'rcup_bot',
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.1'
        ],
        output='screen'
    )

    # ***********************************************************************************
    
    # --- Spawner Nodes  ---
    spawn_joint_state_broadcaster = Node( 
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    spawn_mecanum_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecc_cont', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Event handler to spawn controllers after the robot is spawned
    spawn_controllers_after_robot = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[
                TimerAction(
                    period=4.0,  
                    actions=[spawn_joint_state_broadcaster]
                ),
                TimerAction(
                    period=5.0,  
                    actions=[spawn_mecanum_controller]
                )
            ]
        )
    )



    return LaunchDescription([
        # Launch arguments
        declare_x_pose_arg,
        declare_y_pose_arg,
        world_arg,
        declare_on_exit_shutdown_arg,

        # Start Gazebo
        gzserver_cmd,
        gzclient_cmd,

        spawn_entity_node,
        # # Core nodes
        rsp,
        
        # spawn_controllers_after_robot,
    ]
    )