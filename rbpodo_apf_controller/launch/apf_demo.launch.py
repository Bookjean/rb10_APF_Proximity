import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    apf_pkg = get_package_share_directory("rbpodo_apf_controller")

    model_path = os.path.join(apf_pkg, "urdf", "rb10_1300e_apf.urdf.xacro")
    rviz_config = os.path.join(apf_pkg, "config", "apf_rviz.rviz")
    apf_params = os.path.join(apf_pkg, "config", "apf_params.yaml")
    controllers_yaml = os.path.join(
        get_package_share_directory("rbpodo_bringup"), "config", "controllers.yaml"
    )

    obstacle_sim_enabled = LaunchConfiguration("obstacle_sim_enabled")

    robot_description = Command([
        FindExecutable(name="xacro"), " ", model_path,
        " use_fake_hardware:=true",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "obstacle_sim_enabled",
            default_value="false",
            description="Enable virtual obstacle simulation in fake ToF publisher",
        ),

        # Robot State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="both",
            parameters=[{"robot_description": ParameterValue(robot_description, value_type=str)}],
        ),

        # ros2_control_node
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": ParameterValue(robot_description, value_type=str)},
                controllers_yaml,
            ],
            remappings=[
                ("joint_states", "rbpodo/joint_states"),
            ],
            output="both",
            on_exit=Shutdown(),
        ),

        # Joint State Broadcaster
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen",
        ),

        # Position Controllers
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["position_controllers"],
            output="screen",
        ),

        # Joint State Publisher (relay from rbpodo/joint_states to /joint_states)
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            parameters=[{"source_list": ["rbpodo/joint_states"], "rate": 30}],
        ),

        # APF Controller
        Node(
            package="rbpodo_apf_controller",
            executable="apf_controller",
            name="apf_controller",
            output="screen",
            parameters=[apf_params],
        ),

        # Fake ToF Publisher
        Node(
            package="rbpodo_apf_controller",
            executable="fake_tof_publisher",
            name="fake_tof_publisher",
            output="screen",
            parameters=[
                apf_params,
                {"obstacle_sim_enabled": obstacle_sim_enabled},
            ],
        ),

        # RViz
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["--display-config", rviz_config],
            output="screen",
        ),
    ])
