import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
)
from launch_ros.actions import Node


def generate_launch_description():
    description_pkg = get_package_share_directory("rbpodo_description")

    model_path = os.path.join(description_pkg, "robots", "rb10_1300e.urdf.xacro")
    rviz_config = os.path.join(description_pkg, "rviz", "urdf.rviz")

    gui = LaunchConfiguration("gui")

    robot_description = Command([FindExecutable(name="xacro"), " ", model_path])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                choices=["true", "false"],
                description="Enable joint_state_publisher_gui",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                condition=IfCondition(gui),
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                condition=UnlessCondition(gui),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["--display-config", rviz_config],
                output="screen",
            ),
        ]
    )
