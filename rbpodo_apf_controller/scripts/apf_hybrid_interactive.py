#!/usr/bin/env python3
"""Interactive launcher for APF hybrid controller with simulation mode and sensor selection.

This script should be run directly, not via ros2 launch:
    python3 apf_hybrid_interactive.py
    or
    ./apf_hybrid_interactive.py
"""

import os
import sys
import subprocess

from ament_index_python.packages import get_package_share_directory


def print_banner():
    """Print welcome banner."""
    print("\n" + "=" * 60)
    print("  RB10 APF Hybrid Controller - Interactive Launcher")
    print("=" * 60 + "\n")


def select_simulation_mode():
    """Interactive simulation mode selection."""
    print("로봇 모드를 선택하세요:")
    print()
    print("  1. RViz 시뮬레이션 전용 모드")
    print("     - ros2_control fake_hardware 사용")
    print("     - RViz에서만 로봇 움직임")
    print("     - 실제 로봇은 움직이지 않음")
    print()
    print("  2. 실제 로봇 + RViz 시뮬레이션 모드")
    print("     - 실제 로봇과 RViz 시뮬레이션 모두 제어")
    print("     - 두 로봇이 동기화되어 움직임")
    print()
    
    while True:
        try:
            choice = input("선택 (1 또는 2, 기본값: 1): ").strip()
            if not choice:
                choice = "1"
            
            if choice == "1":
                return "rviz_sim", "RViz 시뮬레이션 전용"
            elif choice == "2":
                return "real", "실제 로봇 + RViz 시뮬레이션"
            else:
                print("잘못된 선택입니다. 1 또는 2를 입력하세요.")
        except (EOFError, KeyboardInterrupt):
            print("\n취소되었습니다.")
            sys.exit(0)


def select_sensor_mode():
    """Interactive sensor mode selection."""
    print("\n센서 모드를 선택하세요:")
    print()
    print("  1. 가짜 ToF 센서 모드 (시뮬레이션)")
    print("     - /link3_tof_*/range 토픽 사용")
    print("     - fake_tof_publisher 노드 실행")
    print("     - 장애물 시뮬레이션 가능")
    print()
    print("  2. 실제 센서 모드 (실제 하드웨어)")
    print("     - 실제 센서 하드웨어 데이터 사용")
    print("     - /proximity_distance1-4 토픽 구독")
    print()
    print("  3. 실제 센서 모드 (테스트용 가짜 발행자)")
    print("     - fake_proximity_publisher 사용")
    print("     - 실제 센서 없이 테스트 가능")
    print()
    
    while True:
        try:
            choice = input("선택 (1, 2, 또는 3, 기본값: 2): ").strip()
            if not choice:
                choice = "2"
            
            if choice == "1":
                return False, False, "가짜 ToF 센서 모드"
            elif choice == "2":
                return True, False, "실제 센서 모드 (하드웨어)"
            elif choice == "3":
                return True, True, "실제 센서 모드 (테스트용)"
            else:
                print("잘못된 선택입니다. 1, 2, 또는 3을 입력하세요.")
        except (EOFError, KeyboardInterrupt):
            print("\n취소되었습니다.")
            sys.exit(0)


def select_obstacle_sim():
    """Interactive obstacle simulation selection (only for fake sensor mode)."""
    print()
    print("장애물 시뮬레이션을 활성화하시겠습니까?")
    print("  (가짜 센서 모드에서만 사용 가능)")
    print()
    
    while True:
        try:
            choice = input("활성화 (y/n, 기본값: n): ").strip().lower()
            if not choice:
                choice = "n"
            
            if choice in ["y", "yes", "예"]:
                return True
            elif choice in ["n", "no", "아니오"]:
                return False
            else:
                print("잘못된 선택입니다. y 또는 n을 입력하세요.")
        except (EOFError, KeyboardInterrupt):
            print("\n취소되었습니다.")
            sys.exit(0)


def select_proximity_sensor_type(use_fake_publisher):
    """Select proximity sensor topic type (only for real sensor mode)."""
    if use_fake_publisher:
        # 테스트 모드에서는 개별 센서 토픽 사용
        return True, "[1, 2, 3, 4]"
    
    print()
    print("실제 센서 토픽 타입을 선택하세요:")
    print()
    print("  1. 개별 센서 토픽 (/proximity_distance1-4)")
    print("     - 각 센서의 Range 타입 토픽 구독")
    print("     - 센서 ID 매핑 필요")
    print()
    print("  2. 통합 토픽 (/proximity_distance)")
    print("     - Float32MultiArray 타입 토픽 구독")
    print("     - 모든 센서 데이터가 하나의 토픽에")
    print()
    
    while True:
        try:
            choice = input("선택 (1 또는 2, 기본값: 1): ").strip()
            if not choice:
                choice = "1"
            
            if choice == "1":
                print()
                print("센서 ID 매핑을 입력하세요 (예: 1,2,3,4 또는 2,3,4,5)")
                print("형식: [N, S, E, W] 방향에 해당하는 센서 ID")
                mapping_input = input("센서 매핑 (기본값: 1,2,3,4): ").strip()
                if not mapping_input:
                    mapping_input = "1,2,3,4"
                
                # Parse input like "1,2,3,4" to "[1, 2, 3, 4]"
                try:
                    sensor_ids = [int(x.strip()) for x in mapping_input.split(",")]
                    mapping_str = "[" + ", ".join(map(str, sensor_ids)) + "]"
                    return True, mapping_str
                except ValueError:
                    print("잘못된 형식입니다. 기본값 [1, 2, 3, 4]를 사용합니다.")
                    return True, "[1, 2, 3, 4]"
            elif choice == "2":
                return False, "[1, 2, 3, 4]"
            else:
                print("잘못된 선택입니다. 1 또는 2를 입력하세요.")
        except (EOFError, KeyboardInterrupt):
            print("\n취소되었습니다.")
            sys.exit(0)


def select_trajectory_mode():
    """Select whether to use trajectory manager."""
    print()
    print("Trajectory Manager를 사용하시겠습니까?")
    print("  (YAML 파일에서 waypoint를 로드하여 순차적으로 실행)")
    print()
    
    while True:
        try:
            choice = input("사용 (y/n, 기본값: n): ").strip().lower()
            if not choice:
                choice = "n"
            
            if choice in ["y", "yes", "예"]:
                return True
            elif choice in ["n", "no", "아니오"]:
                return False
            else:
                print("잘못된 선택입니다. y 또는 n을 입력하세요.")
        except (EOFError, KeyboardInterrupt):
            print("\n취소되었습니다.")
            sys.exit(0)


def main():
    """Main function."""
    print_banner()
    
    # Select simulation mode
    robot_mode, sim_mode_name = select_simulation_mode()
    print(f"\n선택된 모드: {sim_mode_name}")
    
    # Get robot IP (needed for real mode)
    robot_ip = "192.168.111.50"  # Changed to match robotory_rb10_ros2 default
    if robot_mode == "real":
        print()
        robot_ip_input = input(f"로봇 IP 주소 (기본값: {robot_ip}): ").strip()
        if robot_ip_input:
            robot_ip = robot_ip_input
    
    # Convert to simulation_mode boolean for launch file
    # rviz_sim -> true, real -> false
    simulation_mode_for_launch = (robot_mode == "rviz_sim")
    
    # Select sensor mode
    use_real_sensor, use_fake_proximity_publisher, sensor_mode_name = select_sensor_mode()
    print(f"\n선택된 센서 모드: {sensor_mode_name}")
    
    # Select obstacle simulation (only for fake sensor mode)
    obstacle_sim_enabled = False
    if not use_real_sensor:
        obstacle_sim_enabled = select_obstacle_sim()
        if obstacle_sim_enabled:
            print("장애물 시뮬레이션: 활성화")
        else:
            print("장애물 시뮬레이션: 비활성화")
    
    # Select proximity sensor type (only for real sensor mode)
    use_individual_proximity_sensors = True
    proximity_sensor_mapping = "[1, 2, 3, 4]"
    if use_real_sensor:
        use_individual_proximity_sensors, proximity_sensor_mapping = select_proximity_sensor_type(use_fake_proximity_publisher)
        if use_individual_proximity_sensors:
            print(f"개별 센서 토픽 사용: {proximity_sensor_mapping}")
        else:
            print("통합 토픽 (/proximity_distance) 사용")
    
    # Trajectory Manager는 별도로 실행하도록 변경
    print("\nTrajectory Manager는 별도 터미널에서 실행하시겠습니까?")
    print("  (런치 파일에는 포함되지 않습니다)")
    print()
    use_trajectory_separate = select_trajectory_mode()
    trajectory_file = ""
    
    if use_trajectory_separate:
        print()
        print("Trajectory YAML 파일 경로를 입력하세요 (Enter로 기본값 사용)")
        try:
            pkg = get_package_share_directory("rbpodo_apf_controller")
            default_file = os.path.join(pkg, "config", "example_trajectory.yaml")
            if not os.path.exists(default_file):
                default_file = ""
        except Exception as e:
            default_file = ""
        
        file_input = input(f"YAML 파일 경로 [{default_file}]: ").strip()
        if file_input:
            trajectory_file = file_input
        elif default_file:
            trajectory_file = default_file
    
    print("\n" + "=" * 60)
    print("설정 요약:")
    print("=" * 60)
    print(f"  로봇 모드: {sim_mode_name}")
    if robot_mode == "real":
        print(f"  로봇 IP: {robot_ip}")
    print(f"  센서 모드: {sensor_mode_name}")
    if not use_real_sensor:
        print(f"  장애물 시뮬레이션: {'활성화' if obstacle_sim_enabled else '비활성화'}")
    if use_real_sensor:
        print(f"  가짜 발행자 사용: {'예' if use_fake_proximity_publisher else '아니오'}")
        print(f"  개별 센서 토픽: {'예' if use_individual_proximity_sensors else '아니오'}")
        if use_individual_proximity_sensors:
            print(f"  센서 매핑: {proximity_sensor_mapping}")
    if use_trajectory_separate:
        print(f"  Trajectory Manager: 별도 실행 예정")
        if trajectory_file:
            print(f"  Trajectory 파일: {trajectory_file}")
        print(f"\n  ════════════════════════════════════════════════════════")
        print(f"  별도 터미널에서 다음 명령어를 실행하세요:")
        print(f"  ════════════════════════════════════════════════════════")
        if trajectory_file:
            print(f"  ros2 run rbpodo_apf_controller trajectory_manager \\")
            print(f"    --ros-args \\")
            print(f"    -p trajectory_file:={trajectory_file} \\")
            print(f"    -p interactive_mode:=true")
        else:
            print(f"  ros2 run rbpodo_apf_controller trajectory_manager \\")
            print(f"    --ros-args \\")
            print(f"    -p interactive_mode:=true")
        print(f"  ════════════════════════════════════════════════════════")
    else:
        print(f"  Trajectory Manager: 사용 안 함")
    print("=" * 60)
    print("\nAPF 컨트롤러가 실행됩니다...")
    print("런치 파일을 실행합니다...\n")
    
    # Get package share directory
    try:
        apf_pkg = get_package_share_directory("rbpodo_apf_controller")
        launch_file = os.path.join(apf_pkg, "launch", "apf_hybrid.launch.py")
    except Exception as e:
        print(f"패키지를 찾을 수 없습니다: {e}")
        print("\n먼저 워크스페이스를 빌드하고 source 해주세요:")
        print("  cd ~/rb_ws")
        print("  colcon build --packages-select rbpodo_apf_controller")
        print("  source install/setup.bash")
        print("\n그 다음 다시 실행하세요:")
        print("  python3 ~/rb_ws/src/rb10_APF_Proximity/rbpodo_apf_controller/scripts/apf_hybrid_interactive.py")
        sys.exit(1)
    
    # Build launch command
    cmd = [
        "ros2",
        "launch",
        "rbpodo_apf_controller",
        "apf_hybrid.launch.py",
        f"robot_mode:={robot_mode}",
        f"simulation_mode:={'true' if simulation_mode_for_launch else 'false'}",
        f"robot_ip:={robot_ip}",
        f"use_rviz:=true",  # Always enable RViz
        f"use_real_sensor:={'true' if use_real_sensor else 'false'}",
        f"use_fake_proximity_publisher:={'true' if use_fake_proximity_publisher else 'false'}",
        f"obstacle_sim_enabled:={'true' if obstacle_sim_enabled else 'false'}",
        f"use_individual_proximity_sensors:={'true' if use_individual_proximity_sensors else 'false'}",
        f"proximity_sensor_mapping:={proximity_sensor_mapping}",
        f"use_trajectory:=false",  # Launch 파일에는 포함하지 않음 (별도 실행)
        f"interactive_trajectory:=false",
    ]
    
    if trajectory_file:
        cmd.append(f"trajectory_file:={trajectory_file}")
    
    # Execute launch
    try:
        subprocess.run(cmd, check=True)
    except KeyboardInterrupt:
        print("\n\n런치가 중단되었습니다.")
        sys.exit(0)
    except subprocess.CalledProcessError as e:
        print(f"\n런치 실행 중 오류가 발생했습니다: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()

