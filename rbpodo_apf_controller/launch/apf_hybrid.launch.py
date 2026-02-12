"""Hybrid launch file for simulation-only or real+sim mode with APF.

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
    apf_pkg = get_package_share_directory("rbpodo_apf_controller")
    bringup_pkg = get_package_share_directory("rbpodo_bringup")
    desc_pkg = get_package_share_directory("rbpodo_description")

    model_path = os.path.join(apf_pkg, "urdf", "rb10_1300e_apf.urdf.xacro")
    apf_params = os.path.join(apf_pkg, "config", "apf_params.yaml")
    rviz_config = os.path.join(apf_pkg, "config", "apf_rviz.rviz")
    controllers_yaml = os.path.join(bringup_pkg, "config", "controllers.yaml")

    # Launch arguments
    robot_mode = LaunchConfiguration("robot_mode")  # "rviz_sim", "real"
    simulation_mode = LaunchConfiguration("simulation_mode")  # true for rviz_sim, false for real
    robot_ip = LaunchConfiguration("robot_ip")
    model_id = LaunchConfiguration("model_id")
    use_rviz = LaunchConfiguration("use_rviz")
    use_real_sensor = LaunchConfiguration("use_real_sensor")
    use_fake_proximity_publisher = LaunchConfiguration("use_fake_proximity_publisher")
    obstacle_sim_enabled = LaunchConfiguration("obstacle_sim_enabled")
    use_individual_proximity_sensors = LaunchConfiguration("use_individual_proximity_sensors")
    proximity_sensor_mapping = LaunchConfiguration("proximity_sensor_mapping")
    trajectory_file = LaunchConfiguration("trajectory_file")
    use_trajectory = LaunchConfiguration("use_trajectory")

    # Robot description for simulation
    robot_description_sim = Command([
        FindExecutable(name="xacro"), " ", model_path,
        " use_fake_hardware:=true",
    ])

    # Simulation robot setup (for RViz visualization - only for rviz_sim and real modes)
    sim_nodes = [
        # Robot State Publisher (RViz visualization)
        # - rviz_sim: consumes `/joint_states` (relayed from ros2_control)
        # - real: consumes `/joint_states_mapped` (mapped from real robot joint_1~6)
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="both",
            parameters=[{
                "robot_description": ParameterValue(
                    robot_description_sim, value_type=str
                ),
                "use_sim_time": False,
            }],
            condition=IfCondition(PythonExpression([
                "'", robot_mode, "' == 'rviz_sim'"
            ])),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="both",
            parameters=[{
                "robot_description": ParameterValue(
                    robot_description_sim, value_type=str
                ),
                "use_sim_time": False,
            }],
            remappings=[
                ("joint_states", "joint_states_mapped"),
            ],
            condition=IfCondition(PythonExpression([
                "'", robot_mode, "' == 'real'"
            ])),
        ),

        # ros2_control_node (simulation)
        # Only for rviz_sim mode
        # For real mode: use rbpodo_bringup's ros2_control_node (no separate simulation node)
        # Note: name must be 'controller_manager' for spawner to find it
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            name="controller_manager",
            parameters=[
                {
                    "robot_description": ParameterValue(
                        robot_description_sim, value_type=str
                    ),
                },
                controllers_yaml,
            ],
            remappings=[
                ("joint_states", "rbpodo/joint_states"),
            ],
            output="both",
            on_exit=Shutdown(),
            condition=IfCondition(PythonExpression([
                "'", robot_mode, "' == 'rviz_sim'"
            ])),
        ),

        # Joint State Broadcaster (simulation)
        # Only for rviz_sim mode (real mode uses rbpodo_bringup's controllers)
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen",
            condition=IfCondition(PythonExpression([
                "'", robot_mode, "' == 'rviz_sim'"
            ])),
        ),

        # Position Controllers (simulation)
        # Only for rviz_sim mode (real mode uses rbpodo_bringup's controllers)
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["position_controllers"],
            output="screen",
            condition=IfCondition(PythonExpression([
                "'", robot_mode, "' == 'rviz_sim'"
            ])),
        ),

        # Joint State Mapper (real mode only)
        # Maps joint_1~6 -> base~wrist3 and publishes to /joint_states_mapped (never overwrites /joint_states)
        Node(
            package="rbpodo_apf_controller",
            executable="joint_state_mapper",
            name="joint_state_mapper",
            output="screen",
            condition=IfCondition(PythonExpression([
                "'", robot_mode, "' == 'real'"
            ])),
        ),

        # Joint State Publisher (relay)
        # For rviz_sim mode: relay from ros2_control (rbpodo/joint_states)
        # For real mode: we do NOT relay to /joint_states (avoid collisions with the real publisher)
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            parameters=[{
                "source_list": ["rbpodo/joint_states"],
                "rate": 30,
            }],
            condition=IfCondition(PythonExpression([
                "'", robot_mode, "' == 'rviz_sim'"
            ])),
        ),
    ]

    # Real robot setup (only for real mode)
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "rbpodo.launch.py")
        ),
        launch_arguments={
            "robot_ip": robot_ip,
            "model_id": model_id,
            "use_fake_hardware": "false",
            "cb_simulation": "false",
            "use_rviz": "false",  # We'll use our own rviz
        }.items(),
        condition=IfCondition(PythonExpression([
            "'", robot_mode, "' == 'real'"
        ])),
    )

    # Fake ToF Publisher (only if not using real sensor)
    fake_tof_publisher = Node(
        package="rbpodo_apf_controller",
        executable="fake_tof_publisher",
        name="fake_tof_publisher",
        output="screen",
        parameters=[
            apf_params,
            {"obstacle_sim_enabled": obstacle_sim_enabled},
        ],
        condition=IfCondition(PythonExpression([
            "'", use_real_sensor, "' == 'false'"
        ])),
    )

    # Fake Proximity Publisher (for testing real sensor mode without hardware)
    fake_proximity_publisher = Node(
        package="rbpodo_proximity_controller",
        executable="fake_proximity_publisher",
        name="fake_proximity_publisher",
        output="screen",
        parameters=[
            {"obstacle_sim_enabled": obstacle_sim_enabled},
            {"obstacle_position": [0.3, 0.0, 0.8]},
            {"publish_rate": 100.0},
            {"max_range": 200.0},
            {"default_range": 200.0},
        ],
        condition=IfCondition(PythonExpression([
            "'", use_real_sensor, "' == 'true' and '", use_fake_proximity_publisher, "' == 'true'"
        ])),
    )

    # APF Controller node (always runs)
    apf_controller = Node(
        package="rbpodo_apf_controller",
        executable="apf_controller",
        name="apf_controller",
        output="screen",
        parameters=[
            apf_params,
            {"simulation_mode": simulation_mode},
            {"real_robot_ip": robot_ip},
            {"robot_mode": robot_mode},
            {"use_real_sensor": use_real_sensor},
            {"use_individual_proximity_sensors": use_individual_proximity_sensors},
            {"proximity_sensor_mapping": proximity_sensor_mapping},
        ],
    )

    # Launch arguments for trajectory
    interactive_trajectory = LaunchConfiguration("interactive_trajectory")
    
    # Trajectory Manager (if enabled)
    trajectory_manager = Node(
        package="rbpodo_apf_controller",
        executable="trajectory_manager",
        name="trajectory_manager",
        output="screen",
        parameters=[
            {"trajectory_file": trajectory_file},
            {"default_wait_time": 2.0},  # Wait 2 seconds at each waypoint
            {"interactive_mode": interactive_trajectory},
        ],
        condition=IfCondition(use_trajectory),
    )

    # RViz (if enabled)
    # Always launch RViz for visualization
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["--display-config", rviz_config] if os.path.exists(rviz_config) else [],
        # Always launch RViz (use_rviz parameter is for future use)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_mode",
            default_value="rviz_sim",
            description="Robot mode: 'rviz_sim' (RViz simulation only), 'real' (real robot + RViz)",
        ),
        DeclareLaunchArgument(
            "simulation_mode",
            default_value="true",
            description="Legacy parameter: If true, only simulation robot moves. If false, both real and sim move. (Use robot_mode instead)",
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.111.50",  # Changed to match robotory_rb10_ros2 default
            description="Real robot IP address",
        ),
        DeclareLaunchArgument(
            "model_id",
            default_value="rb10_1300e",
            description="Robot model ID",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz",
        ),
        DeclareLaunchArgument(
            "use_real_sensor",
            default_value="true",
            description=(
                "센서 모드 선택: "
                "true = 실제 센서 데이터(/proximity_distance 토픽) 사용, "
                "false = 가짜 ToF 센서 데이터(/link3_tof_*/range 토픽) 사용"
            ),
        ),
        DeclareLaunchArgument(
            "use_fake_proximity_publisher",
            default_value="false",
            description=(
                "실제 센서 모드에서도 테스트용 fake_proximity_publisher 사용: "
                "true = fake_proximity_publisher 실행 (실제 센서가 없을 때 테스트용), "
                "false = 실제 센서 하드웨어 사용"
            ),
        ),
        DeclareLaunchArgument(
            "obstacle_sim_enabled",
            default_value="false",
            description="Enable virtual obstacle simulation in fake ToF publisher",
        ),
        DeclareLaunchArgument(
            "use_individual_proximity_sensors",
            default_value="true",
            description=(
                "개별 센서 토픽 사용: "
                "true = /proximity_distance1-4 토픽 구독 (Range 타입), "
                "false = /proximity_distance 토픽 구독 (Float32MultiArray 타입)"
            ),
        ),
        DeclareLaunchArgument(
            "proximity_sensor_mapping",
            default_value="[1, 2, 3, 4]",
            description=(
                "센서 ID 매핑 [N, S, E, W]: "
                "예: [1, 2, 3, 4] = proximity_distance1→N, proximity_distance2→S, ..."
            ),
        ),
        DeclareLaunchArgument(
            "trajectory_file",
            default_value="",
            description="Path to YAML file containing waypoint positions and trajectories",
        ),
        DeclareLaunchArgument(
            "use_trajectory",
            default_value="false",
            description="Enable trajectory manager for point-to-point trajectory execution",
        ),
        DeclareLaunchArgument(
            "interactive_trajectory",
            default_value="false",  # Disabled by default - use ROS2 services instead
            description="Enable interactive menu for trajectory selection (only if use_trajectory=true). Note: Requires stdin access, may not work via ros2 launch. Use ROS2 services instead.",
        ),

        # Simulation nodes (for rviz_sim and real modes)
        *sim_nodes,

        # Real robot nodes (only for real mode)
        bringup_launch,

        # Fake sensor publishers (conditional)
        fake_tof_publisher,
        fake_proximity_publisher,

        # APF controller (always runs)
        apf_controller,

        # Trajectory manager
        trajectory_manager,

        # RViz
        rviz_node,
    ])

