from typing import Optional,List
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart, OnProcessExit
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
    arg_use_mock_hardware = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="true",
        choices=["true", "false"],
        description="true = simulation/fake hardware, false = real hardware",
    )
    arg_launch_rviz = DeclareLaunchArgument(
        "launch_rviz",
        default_value="false",
        choices=["true", "false"],
        description="Launch the regular URDF RViz (not MoveIt RViz)",
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

    # Launch Configurations
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    rviz_file = PathJoinSubstitution([FindPackageShare(description_package), 'rviz', 'urdf.rviz'])
    controller_yaml_file = PathJoinSubstitution([FindPackageShare("smart_template_cpp"), "config", "smart_template_controllers.yaml"])
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")

    # Get Robot description URDF via xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(description_package), 'urdf', description_file]),
        " ",
        "name:=", LaunchConfiguration('name'),
        " ",
        "use_mock_hardware:=", LaunchConfiguration('use_mock_hardware'),
        " ",
        "robot_mode:=", LaunchConfiguration('robot_mode'),
        " ",
        "needle_type:=", LaunchConfiguration('needle_type'),
        " ",
        "zframe_config:=", LaunchConfiguration('zframe_config')
    ])
    robot_description = {
        "robot_description": robot_description_content
    }
    
    # Nodes
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
        parameters=[
            robot_description,
            {"publish_frequency": 30.0}
        ],
    )

    tf_world = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='tf_world',
    output='screen',
    arguments=[
            '--x','0','--y','0','--z','0',
            '--qx','0','--qy','0','--qz','0','--qw','1',
            '--frame-id','world',
            '--child-frame-id','base_link',
            ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        name='rviz2',
        output='screen'
    )

    world_pose_node = Node(
        package='smart_template_cpp',
        executable='world_pose_listener',
        name='world_pose_listener',
        output='screen',
    )

    # Include launch arguments
    ld.add_action(arg_use_mock_hardware)
    ld.add_action(arg_robot_mode)
    ld.add_action(arg_needle_type)
    ld.add_action(arg_zframe_config)
    ld.add_action(arg_launch_rviz)
    ld.add_action(arg_gui)
    ld.add_action(arg_description_package)
    ld.add_action(arg_description_file)
    ld.add_action(arg_name)
    ld.add_action(arg_controller_spawner_timeout)
    

    # Base nodes
    ld.add_action(robot_state_publisher_node)
    ld.add_action(tf_world)
    ld.add_action(world_pose_node)
    ld.add_action(rviz_node)

    # Start control stack after robot_state_publisher starts
    ld.add_action(
        RegisterEventHandler(
            OnProcessStart(
                target_action = robot_state_publisher_node,
                on_start = [control_node,
                           smart_template_node,
                           ExecuteProcess(
                                condition=IfCondition(LaunchConfiguration('gui')),
                                cmd=['rqt', '--standalone', 'smart_template_gui', '--force-discover'],
                                output='screen'
                           )]
            )
        )
    )

    # Spawn controllers    
    def controller_spawner(
        controller_name: str,
        active: bool = True,
        controller_type: str = None,
        condition: Optional[LaunchConfiguration] = None,
        extra_args: List[str] = None,
    ):
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
            output="screen",
        )

    #pc_spawner = controller_spawner("position_controller", active=True)
    jtc_spawner = controller_spawner("joint_trajectory_controller", active=True)
    jsb_spawner = controller_spawner("joint_state_broadcaster", active=True)

    ld.add_action(jsb_spawner)
    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=jsb_spawner,
                on_exit=[
                    TimerAction(
                        period=1.0,
                        actions=[jtc_spawner],
                    )
                ],
            )
        )
    )

    ld.add_action(LogInfo(msg=["[robot_launch] Active controller = joint_trajectory_controller"]))

    return ld