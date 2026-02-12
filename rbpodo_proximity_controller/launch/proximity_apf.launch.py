"""Unified launch file for proximity APF controller.

robot_mode='sim':  fake hardware + fake_proximity_publisher + RViz
robot_mode='real': rbpodo_bringup (real robot) + real sensors + RViz

RViz is always launched regardless of mode.

Optional: waypoint_file + auto_execute for automatic trajectory execution.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('rbpodo_proximity_controller')
    bringup_pkg = get_package_share_directory('rbpodo_bringup')
    desc_pkg = get_package_share_directory('rbpodo_description')

    model_path = os.path.join(desc_pkg, 'robots', 'rb10_1300e.urdf.xacro')
    controller_params = os.path.join(pkg, 'config', 'controller_params.yaml')
    controllers_yaml = os.path.join(
        bringup_pkg, 'config', 'controllers.yaml'
    )
    rviz_config = os.path.join(pkg, 'config', 'proximity_demo.rviz')

    # Launch arguments
    robot_mode = LaunchConfiguration('robot_mode')
    robot_ip = LaunchConfiguration('robot_ip')
    model_id = LaunchConfiguration('model_id')
    obstacle_sim_enabled = LaunchConfiguration('obstacle_sim_enabled')
    waypoint_file = LaunchConfiguration('waypoint_file')
    auto_execute = LaunchConfiguration('auto_execute')
    use_rviz = LaunchConfiguration('use_rviz')

    # Robot description with fake hardware (for sim and RViz visualization)
    robot_description_sim = Command([
        FindExecutable(name='xacro'), ' ', model_path,
        ' use_fake_hardware:=true',
    ])

    is_sim = PythonExpression(["'", robot_mode, "' == 'sim'"])
    is_real = PythonExpression(["'", robot_mode, "' == 'real'"])

    # ---------------------------------------------------------------
    # SIM-ONLY NODES (robot_mode == 'sim')
    # ---------------------------------------------------------------
    sim_nodes = [
        # Robot State Publisher
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
            condition=IfCondition(is_sim),
        ),

        # ros2_control_node (fake hardware)
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
            condition=IfCondition(is_sim),
        ),

        # Joint State Broadcaster (sim)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
            condition=IfCondition(is_sim),
        ),

        # Position Controllers (sim)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['position_controllers'],
            output='screen',
            condition=IfCondition(is_sim),
        ),

        # Joint State Publisher relay (sim)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{
                'source_list': ['rbpodo/joint_states'],
                'rate': 30,
            }],
            condition=IfCondition(is_sim),
        ),

        # Fake Proximity Publisher (sim only)
        Node(
            package='rbpodo_proximity_controller',
            executable='fake_proximity_publisher',
            name='fake_proximity_publisher',
            output='screen',
            parameters=[
                controller_params,
                {'obstacle_sim_enabled': obstacle_sim_enabled},
            ],
            condition=IfCondition(is_sim),
        ),
    ]

    # ---------------------------------------------------------------
    # REAL-ONLY NODES (robot_mode == 'real')
    # ---------------------------------------------------------------
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'rbpodo.launch.py')
        ),
        launch_arguments={
            'robot_ip': robot_ip,
            'model_id': model_id,
            'use_fake_hardware': 'false',
            'cb_simulation': 'false',
            'use_rviz': 'false',
        }.items(),
        condition=IfCondition(is_real),
    )

    real_nodes = [
        # Robot State Publisher (real) - uses mapped joint states for RViz
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
            remappings=[
                ('joint_states', 'joint_states_mapped'),
            ],
            condition=IfCondition(is_real),
        ),

        # Joint State Mapper (real only) - joint_1~6 -> base~wrist3
        Node(
            package='rbpodo_proximity_controller',
            executable='joint_state_mapper',
            name='joint_state_mapper',
            output='screen',
            condition=IfCondition(is_real),
        ),
    ]

    # ---------------------------------------------------------------
    # ALWAYS-RUNNING NODES
    # ---------------------------------------------------------------
    # Proximity Controller
    proximity_controller = Node(
        package='rbpodo_proximity_controller',
        executable='proximity_controller',
        name='proximity_controller',
        output='screen',
        parameters=[
            controller_params,
            {'auto_execute': auto_execute},
        ],
    )

    # Waypoint Manager
    waypoint_manager = Node(
        package='rbpodo_proximity_controller',
        executable='waypoint_manager',
        name='waypoint_manager',
        output='screen',
        parameters=[
            controller_params,
            {'waypoint_file': waypoint_file},
        ],
    )

    # RViz (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=(
            ['-d', rviz_config]
            if os.path.exists(rviz_config)
            else []
        ),
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_mode',
            default_value='sim',
            description=(
                "Robot mode: 'sim' (fake hardware + fake sensors) "
                "or 'real' (real robot + real sensors)"
            ),
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
            'obstacle_sim_enabled',
            default_value='false',
            description='Enable virtual obstacle simulation (sim mode only)',
        ),
        DeclareLaunchArgument(
            'waypoint_file',
            default_value='waypoints.yaml',
            description='Waypoint YAML file name or full path',
        ),
        DeclareLaunchArgument(
            'auto_execute',
            default_value='false',
            description='Auto-execute waypoints when loaded',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz',
        ),

        # Sim nodes
        *sim_nodes,

        # Real robot bringup
        bringup_launch,

        # Real-only nodes
        *real_nodes,

        # Always-running nodes
        proximity_controller,
        waypoint_manager,
        rviz_node,
    ])
