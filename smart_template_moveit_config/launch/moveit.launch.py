from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    arg_robot_mode = DeclareLaunchArgument(
        "robot_mode",
        default_value="default",
        description="default / calibration",
    )
    arg_needle_type = DeclareLaunchArgument(
        "needle_type",
        default_value="default",
        description="default / stylet / smartneedle",
    )
    arg_zframe_config = DeclareLaunchArgument(
        "zframe_config",
        default_value="default",
        description="default / test / new / old",
    )
    arg_use_mock_hardware = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="true",
        choices=["true", "false"],
        description="Kept consistent with robot launch; used in xacro expansion",
    )
    arg_launch_rviz = DeclareLaunchArgument(
        "launch_rviz",
        default_value="true",
        choices=["true", "false"],
        description="Launch MoveIt RViz",
    )
    arg_description_package = DeclareLaunchArgument(
        "description_package",
        default_value="smart_template_description",
        description="Description package with robot URDF/xacro files",
    )
    arg_description_file = DeclareLaunchArgument(
        "description_file",
        default_value="smart_template.urdf.xacro",
        description="URDF/XACRO description file with the robot",
    )
    arg_name = DeclareLaunchArgument(
        "name",
        default_value="smart_template",
        description="Name of the robot system",
    )

    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "name:=", LaunchConfiguration("name"),
            " ",
            "use_mock_hardware:=", LaunchConfiguration("use_mock_hardware"),
            " ",
            "robot_mode:=", LaunchConfiguration("robot_mode"),
            " ",
            "needle_type:=", LaunchConfiguration("needle_type"),
            " ",
            "zframe_config:=", LaunchConfiguration("zframe_config"),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    moveit_config = (
        MoveItConfigsBuilder(
            "smart_template",
            package_name="smart_template_moveit_config",
        )
        .to_moveit_configs()
    )
    moveit_config.robot_description = robot_description

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    moveit_rviz_file = PathJoinSubstitution(
        [FindPackageShare("smart_template_moveit_config"), "config", "moveit.rviz"]
    )

    moveit_rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", moveit_rviz_file],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
        parameters=[
            robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    ld = LaunchDescription()
    ld.add_action(arg_robot_mode)
    ld.add_action(arg_needle_type)
    ld.add_action(arg_zframe_config)
    ld.add_action(arg_use_mock_hardware)
    ld.add_action(arg_launch_rviz)
    ld.add_action(arg_description_package)
    ld.add_action(arg_description_file)
    ld.add_action(arg_name)

    ld.add_action(move_group_node)

    ld.add_action(
        TimerAction(
            period=5.0,
            actions=[moveit_rviz_node],
        )
    )

    return ld