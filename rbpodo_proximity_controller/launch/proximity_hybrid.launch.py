"""Hybrid launch file for simulation-only or real+sim mode with proximity APF.

In simulation mode:
- Reads real robot's initial pose and sets simulation robot to match
- Only sends commands to simulation robot, real robot stays still
- Uses proximity sensors for APF obstacle avoidance

In real mode:
- Sends commands to both simulation and real robot
- Both robots move in sync
- Uses proximity sensors for APF obstacle avoidance
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('rbpodo_proximity_controller')
    bringup_pkg = get_package_share_directory('rbpodo_bringup')
    desc_pkg = get_package_share_directory('rbpodo_description')

    model_path = os.path.join(desc_pkg, 'robots', 'rb10_1300e.urdf.xacro')
    controller_params = os.path.join(pkg, 'config', 'controller_params.yaml')
    controllers_yaml = os.path.join(bringup_pkg, 'config', 'controllers.yaml')

    # Launch arguments
    simulation_mode = LaunchConfiguration('simulation_mode')
    robot_ip = LaunchConfiguration('robot_ip')
    model_id = LaunchConfiguration('model_id')
    use_rviz = LaunchConfiguration('use_rviz')

    # Robot description for simulation
    robot_description_sim = Command([
        FindExecutable(name='xacro'), ' ', model_path,
        ' use_fake_hardware:=true',
    ])

    # Robot description for real robot (if needed)
    robot_description_real = Command([
        FindExecutable(name='xacro'), ' ', model_path,
        ' use_fake_hardware:=false',
    ])

    # Simulation robot setup (always needed for visualization)
    sim_nodes = [
        # Robot State Publisher (simulation)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[{
                'robot_description': ParameterValue(
                    robot_description_sim, value_type=str
                ),
                'use_sim_time': False,
            }],
        ),

        # ros2_control_node (simulation)
        # Note: name must be 'controller_manager' for spawner to find it
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            parameters=[
                {
                    'robot_description': ParameterValue(
                        robot_description_sim, value_type=str
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

        # Joint State Broadcaster (simulation)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),

        # Position Controllers (simulation)
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
    ]

    # Real robot setup (only if not in simulation-only mode)
    # We'll use IfCondition to conditionally include this
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'rbpodo.launch.py')
        ),
        launch_arguments={
            'robot_ip': robot_ip,
            'model_id': model_id,
            'use_fake_hardware': 'false',
            'cb_simulation': 'false',
            'use_rviz': 'false',  # We'll use our own rviz
        }.items(),
        condition=IfCondition(PythonExpression([
            "'", simulation_mode, "' == 'false'"
        ])),
    )

    # Hybrid controller node
    hybrid_controller = Node(
        package='rbpodo_proximity_controller',
        executable='sim_real_hybrid_controller',
        name='sim_real_hybrid_controller',
        output='screen',
        parameters=[
            controller_params,
            {'simulation_mode': simulation_mode},
            {'real_robot_ip': robot_ip},
        ],
    )

    # Waypoint Manager
    waypoint_manager = Node(
        package='rbpodo_proximity_controller',
        executable='waypoint_manager',
        name='waypoint_manager',
        output='screen',
        parameters=[controller_params],
    )

    # RViz (if enabled)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg, 'config', 'proximity_demo.rviz')] if os.path.exists(os.path.join(pkg, 'config', 'proximity_demo.rviz')) else [],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'simulation_mode',
            default_value='true',
            description='If true, only simulation robot moves. If false, both real and sim move.',
        ),
        DeclareLaunchArgument(
            'robot_ip',
            default_value='10.0.2.7',
            description='Real robot IP address',
        ),
        DeclareLaunchArgument(
            'model_id',
            default_value='rb10_1300e',
            description='Robot model ID',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz',
        ),
        
        # Simulation nodes (always)
        *sim_nodes,
        
        # Real robot nodes (only if not simulation-only)
        bringup_launch,
        
        # Hybrid controller
        hybrid_controller,
        
        # Waypoint manager
        waypoint_manager,
        
        # RViz
        rviz_node,
    ])

