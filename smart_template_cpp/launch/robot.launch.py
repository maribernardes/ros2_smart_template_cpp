from typing import Optional,List
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.actions import LogInfo, TimerAction, ExecuteProcess

# Launch smart_template robot (hardware or virtual version) with ros2_control 

def generate_launch_description():

    ld = LaunchDescription()

    # Launch arguments
    arg_robot_mode = DeclareLaunchArgument(
        'robot_mode',
        default_value = 'default',
        description = 'default / calibration' 
    )  
    arg_needle_type = DeclareLaunchArgument(
        'needle_type',
        default_value = 'default',
        description = 'default / stylet / smartneedle'
    )  
    arg_zframe_config = DeclareLaunchArgument(
        'zframe_config',
        default_value = 'default',
        description = 'default / test / new / old'
    )  
    arg_sim_level = DeclareLaunchArgument(
        'sim_level',
        default_value = '1',
        description='1 = simulation (fake), 2 = real hardware'
    )  
    arg_rviz = DeclareLaunchArgument(
        'rviz', 
        default_value = 'false', 
        choices = ['true', 'false'],
        description = 'Start RViz automatically'
    )
    arg_gui = DeclareLaunchArgument(
        'gui', 
        default_value = 'true', 
        choices = ['true', 'false'],
        description = 'Start SmartTemplate GUI plugin automatically'
    )
    arg_description_package = DeclareLaunchArgument(
        'description_package',
        default_value = 'smart_template_description',
        description = 'Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.',
    )
    arg_description_file = DeclareLaunchArgument(
        'description_file',
        default_value = 'smart_template.urdf.xacro',
        description = 'URDF/XACRO description file with the robot'
    )
    arg_name = DeclareLaunchArgument(
        'name',
        default_value = 'smart_template',
        description = 'Name of the robot system'
    )
    arg_controller_spawner_timeout = DeclareLaunchArgument(
        "controller_spawner_timeout",
        default_value="10",
        description="Timeout used when spawing controllers"
    )
    arg_initial_controller = DeclareLaunchArgument(
        "initial_controller",
        default_value="position_controller",
        description="Initially loaded robot controller"
    )

    # Launch Configurations
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    robot_description_rate = LaunchConfiguration('rate', default=50.0)  # Hz, default is 10 so we're increasing that a bit. 
    rviz_file = PathJoinSubstitution([FindPackageShare(description_package), 'rviz', 'urdf.rviz'])
    controller_yaml_file = PathJoinSubstitution([FindPackageShare("smart_template_cpp"), "config", "smart_template_controllers.yaml"])
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    initial_controller = LaunchConfiguration("initial_controller")

    # Get Robot description URDF via xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(description_package), 'urdf', description_file]),
        " ",
        "name:=", LaunchConfiguration('name'),
        " ",
        "sim_level:=", LaunchConfiguration('sim_level'),
        " ",
        "robot_mode:=", LaunchConfiguration('robot_mode'),
        " ",
        "needle_type:=", LaunchConfiguration('needle_type'),
        " ",
        "zframe_config:=", LaunchConfiguration('zframe_config')
    ])
    robot_description = {
        "robot_description": robot_description_content, 
        'publish_frequency': robot_description_rate
    }

    # Nodes
    def controller_spawner(controller_name: str, active: bool = True, controller_type: str = None, 
                           condition: Optional[LaunchConfiguration] = None, extra_args: List[str] = None):
        inactive_flags = ["--inactive"] if not active else []
        type_flags = ["--controller-type", controller_type] if controller_type else []
        args = [
            controller_name,
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", controller_spawner_timeout,
        ] + inactive_flags + type_flags
        if extra_args:
            args += extra_args
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=args,
            condition=IfCondition(condition) if condition else None,
            output="screen"
        )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_yaml_file],
        output="screen"
    )

    smart_template_node = Node(
        package="smart_template_cpp",
        executable="smart_template_node",
        name="smart_template_node",
        output="screen",
        parameters=[robot_description]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        condition=IfCondition(LaunchConfiguration('rviz')),
        name='rviz2',
        output='screen'
    )

    world_pose_node = Node(
        package='smart_template_cpp',
        executable='world_pose_listener',
        name='world_pose_listener',
        output='screen',
    )

    # Activate controller
    activate_controller = TimerAction(
        period = 3.0,  # Wait to ensure all spawners are ready
        actions = [
            ExecuteProcess(
                cmd=[
                    "ros2", "control", "switch_controllers",
                    "--activate", initial_controller,
                    "--strict"
                ],
                shell=True,
                output="screen"
            )
        ]
    )

    # Include launch arguments
    ld.add_action(arg_sim_level)
    ld.add_action(arg_robot_mode)
    ld.add_action(arg_needle_type)
    ld.add_action(arg_zframe_config)
    ld.add_action(arg_rviz)
    ld.add_action(arg_gui)
    ld.add_action(arg_description_package)
    ld.add_action(arg_description_file)
    ld.add_action(arg_name)
    ld.add_action(arg_controller_spawner_timeout)
    ld.add_action(arg_initial_controller)
    
    # Nodes
    ld.add_action(robot_state_publisher_node) # Publishes robot_description
    ld.add_action( # Start smart_template, control node and rqt GUI AFTER robot_state_publisher is available
        RegisterEventHandler(
            OnProcessStart(
                target_action = robot_state_publisher_node, # Wait until robot_state_publisher_node is started
                on_start = [control_node,
                            smart_template_node,
                            ExecuteProcess(
                                condition=IfCondition(LaunchConfiguration('gui')),
                                cmd=['rqt', '--standalone', 'smart_template_gui'],
                                output='screen'
                            )]
            )
        )
    )

    ld.add_action(controller_spawner("joint_state_broadcaster", active=True))
    for name in ["position_controller", "velocity_controller"]:
        ld.add_action(controller_spawner(name, active=False))
    ld.add_action( # Activate initial_controller AFTER control node
        RegisterEventHandler(
            OnProcessStart(
                target_action = control_node,  # Wait until control_node is started
                on_start = [ExecuteProcess(
                                cmd=["ros2", "control", "switch_controllers",
                                    "--activate", initial_controller,
                                    "--strict"
                                ],
                                shell=True,
                                output="screen"
                            )]
            )
        )
    )
    ld.add_action(rviz_node)
    ld.add_action(world_pose_node)
    
    # Logging 
    ld.add_action(LogInfo(msg=['[robot_launch] initial_controller = ', initial_controller]))

    return ld




