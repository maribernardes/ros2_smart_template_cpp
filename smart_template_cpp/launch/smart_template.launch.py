import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess, DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, TextSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    # Get the package directory
    smart_template_cpp_dir = get_package_share_directory('smart_template_cpp')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz if true'
    )
    
    declare_use_gui_cmd = DeclareLaunchArgument(
        'use_gui',
        default_value='false',
        description='Launch Smart Template GUI if true'
    )
    
    declare_name_cmd = DeclareLaunchArgument(
        'name',
        default_value='smart_template',
        description='Name of the robot'
    )
    
    declare_needle_type_cmd = DeclareLaunchArgument(
        'needle_type',
        default_value='default',
        description='Type of needle to use (default, biopsygun, smartneedle, stylet)'
    )
    
    declare_zframe_config_cmd = DeclareLaunchArgument(
        'zframe_config',
        default_value='default',
        description='Configuration for the zframe'
    )
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_gui = LaunchConfiguration('use_gui')
    name = LaunchConfiguration('name')
    needle_type = LaunchConfiguration('needle_type')
    zframe_config = LaunchConfiguration('zframe_config')

    # URDF file
    urdf_file = os.path.join(
        smart_template_cpp_dir,
        'description/urdf',
        'smart_template.urdf.xacro'
    )

    # Controllers file
    controllers_file = os.path.join(
        smart_template_cpp_dir,
        'config',
        'smart_template_controllers.yaml'
    )

    # RVIZ configuration
    rviz_config = os.path.join(
        smart_template_cpp_dir,
        'description/rviz',
        'urdf.rviz'
    )
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': ParameterValue(
                Command(['xacro ', urdf_file, ' ', 
                         'name:=', name, ' ',
                         'needle_type:=', needle_type, ' ',
                         'zframe_config:=', zframe_config]), 
                value_type=str
            )},
            {'use_sim_time': use_sim_time}
        ]
    )

    # Controller manager
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': ParameterValue(
                Command(['xacro ', urdf_file, ' ', 
                         'name:=', name, ' ',
                         'needle_type:=', needle_type, ' ',
                         'zframe_config:=', zframe_config]), 
                value_type=str
            )},
            controllers_file
        ],
        output='screen'
    )

    # Controllers to start
    controllers_arg = DeclareLaunchArgument(
        'controllers',
        default_value='["joint_state_broadcaster", "position_controller"]',
        description='Controllers to start'
    )

    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # Position controller
    position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['position_controller', '--controller-manager', '/controller_manager'],
    )

    # Smart template node (for services and actions)
    smart_template_node = Node(
        package='smart_template_cpp',
        executable='smart_template_node',
        name='smart_template_node',
        output='screen'
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz)
    )
    
    # Launch GUI script directly with a delay to give time for services to register
    gui_plugin_process = ExecuteProcess(
        condition=IfCondition(use_gui),
        cmd=['bash', '-c', 'sleep 3 && run_gui.py'],
        output='screen'
    )

    # Delay starting controllers until robot_state_publisher has published the robot description
    broadcaster_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_state_publisher_node,
            on_exit=[joint_state_broadcaster_spawner]
        )
    )

    position_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[position_controller_spawner]
        )
    )

    # Define LaunchDescription and add the actions
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_gui_cmd)
    ld.add_action(declare_name_cmd)
    ld.add_action(declare_needle_type_cmd)
    ld.add_action(declare_zframe_config_cmd)
    ld.add_action(controllers_arg)

    # Add nodes
    ld.add_action(robot_state_publisher_node)
    ld.add_action(controller_manager_node)
    ld.add_action(broadcaster_event)
    ld.add_action(position_controller_event)
    ld.add_action(smart_template_node)
    ld.add_action(rviz_node)
    ld.add_action(gui_plugin_process)

    return ld