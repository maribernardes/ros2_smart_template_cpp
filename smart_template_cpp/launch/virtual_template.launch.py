from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
#from ament_index_python.packages import get_package_share_directory
#from launch.launch_description_sources import PythonLaunchDescriptionSource
#import sys
#import os
#from launch import LaunchDescription, actions


def generate_launch_description():

    robot_description_arg = DeclareLaunchArgument(
        'robot_description',
        description='URDF description of the robot'
    )
    
    # Use smart template node
    virtual_robot = Node(
        package="smart_template_cpp",
        executable="virtual_template",
        parameters=[{
            "robot_description": LaunchConfiguration("robot_description")
        }]
    )  

    return LaunchDescription([
        robot_description_arg,
        virtual_robot
    ])
