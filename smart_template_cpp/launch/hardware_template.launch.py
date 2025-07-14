from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

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

    # Controllers file
    smart_template_cpp_dir = get_package_share_directory('smart_template_cpp')
    controller_yaml = os.path.join(
        smart_template_cpp_dir,
        'config',
        'smart_template_controllers.yaml'
    )

    # Controllers to start
    controllers_arg = DeclareLaunchArgument(
        'controllers',
        default_value='["joint_state_broadcaster", "position_controller"]',
        description='Controllers to start'
    )

    # Controller manager
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            controller_yaml
        ],
        output='screen'
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

    # Use smart template node (for services and actions)
    robot = Node(
        package="smart_template_cpp",
        executable="smart_template_node",
        parameters=[
            {
                "robot_description": ParameterValue(
                    LaunchConfiguration("robot_description"),
                    value_type=str
                )
            }
        ]
    )  

    return LaunchDescription([
        controllers_arg,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        position_controller_spawner,
        robot
    ])
