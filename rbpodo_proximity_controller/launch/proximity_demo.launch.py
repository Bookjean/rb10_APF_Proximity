"""Simulation demo launch for proximity controller with fake hardware."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('rbpodo_proximity_controller')
    bringup_pkg = get_package_share_directory('rbpodo_bringup')
    desc_pkg = get_package_share_directory('rbpodo_description')

    model_path = os.path.join(desc_pkg, 'robots', 'rb10_1300e.urdf.xacro')
    controller_params = os.path.join(pkg, 'config', 'controller_params.yaml')
    controllers_yaml = os.path.join(bringup_pkg, 'config', 'controllers.yaml')

    obstacle_sim_enabled = LaunchConfiguration('obstacle_sim_enabled')

    robot_description = Command([
        FindExecutable(name='xacro'), ' ', model_path,
        ' use_fake_hardware:=true',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'obstacle_sim_enabled',
            default_value='false',
            description='Enable virtual obstacle simulation',
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[{
                'robot_description': ParameterValue(
                    robot_description, value_type=str
                ),
                'use_sim_time': False,
            }],
        ),

        # ros2_control_node
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {
                    'robot_description': ParameterValue(
                        robot_description, value_type=str
                    ),
                },
                controllers_yaml,
            ],
            remappings=[
                ('joint_states', 'rbpodo/joint_states'),
            ],
            output='both',
            on_exit=Shutdown(),
        ),

        # Joint State Broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),

        # Position Controllers
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['position_controllers'],
            output='screen',
        ),

        # Joint State Mapper - converts joint_1~6 to base~wrist3
        Node(
            package='rbpodo_proximity_controller',
            executable='joint_state_mapper',
            name='joint_state_mapper',
            output='screen',
        ),
        
        # Joint State Publisher (relay) - uses mapped joint states
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{
                'source_list': ['/joint_states_mapped'],
                'rate': 30,
            }],
        ),

        # Proximity Controller
        Node(
            package='rbpodo_proximity_controller',
            executable='proximity_controller',
            name='proximity_controller',
            output='screen',
            parameters=[controller_params],
        ),

        # Waypoint Manager
        Node(
            package='rbpodo_proximity_controller',
            executable='waypoint_manager',
            name='waypoint_manager',
            output='screen',
            parameters=[controller_params],
        ),

        # Fake Proximity Publisher
        Node(
            package='rbpodo_proximity_controller',
            executable='fake_proximity_publisher',
            name='fake_proximity_publisher',
            output='screen',
            parameters=[
                controller_params,
                {'obstacle_sim_enabled': obstacle_sim_enabled},
            ],
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg, 'config', 'proximity_demo.rviz')] if os.path.exists(os.path.join(pkg, 'config', 'proximity_demo.rviz')) else [],
        ),
    ])
