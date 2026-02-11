"""Real robot launch for proximity controller.

Includes rbpodo_bringup for hardware interface, then adds
proximity controller and waypoint manager on top.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('rbpodo_proximity_controller')
    bringup_pkg = get_package_share_directory('rbpodo_bringup')

    controller_params = os.path.join(pkg, 'config', 'controller_params.yaml')

    robot_ip = LaunchConfiguration('robot_ip')
    model_id = LaunchConfiguration('model_id')

    # Include rbpodo_bringup (hardware interface + controllers)
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'rbpodo.launch.py')
        ),
        launch_arguments={
            'robot_ip': robot_ip,
            'model_id': model_id,
            'use_fake_hardware': 'false',
            'cb_simulation': 'false',
            'use_rviz': 'true',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ip',
            default_value='10.0.2.7',
            description='Robot IP address',
        ),
        DeclareLaunchArgument(
            'model_id',
            default_value='rb10_1300e',
            description='Robot model ID',
        ),

        # Bringup (hardware + controllers + rviz)
        bringup_launch,

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
    ])
