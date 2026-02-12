import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import conditions
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PythonExpression
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
    use_real_sensor = LaunchConfiguration("use_real_sensor")
    use_fake_proximity_publisher = LaunchConfiguration("use_fake_proximity_publisher")
    use_individual_proximity_sensors = LaunchConfiguration("use_individual_proximity_sensors")
    proximity_sensor_mapping = LaunchConfiguration("proximity_sensor_mapping")

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
        DeclareLaunchArgument(
            "use_real_sensor",
            default_value="true",  # 기본값을 true로 변경 (실제 센서 사용 시 편리)
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
            "use_individual_proximity_sensors",
            default_value="true",  # 기본값을 true로 변경 (실제 센서 사용 시 편리)
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
            parameters=[
                apf_params,
                {"use_real_sensor": use_real_sensor},
                {"use_individual_proximity_sensors": use_individual_proximity_sensors},
                {"proximity_sensor_mapping": proximity_sensor_mapping},
            ],
        ),

        # Fake ToF Publisher (only if not using real sensor)
        Node(
            package="rbpodo_apf_controller",
            executable="fake_tof_publisher",
            name="fake_tof_publisher",
            output="screen",
            parameters=[
                apf_params,
                {"obstacle_sim_enabled": obstacle_sim_enabled},
            ],
            condition=conditions.UnlessCondition(use_real_sensor),
        ),

        # Fake Proximity Publisher (for testing real sensor mode without hardware)
        Node(
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
