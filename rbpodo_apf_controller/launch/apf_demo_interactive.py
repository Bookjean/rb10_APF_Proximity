#!/usr/bin/env python3
"""Interactive launcher for APF demo with sensor mode selection."""

import os
import sys
import subprocess

from ament_index_python.packages import get_package_share_directory


def print_banner():
    """Print welcome banner."""
    print("\n" + "=" * 60)
    print("  RB10 APF Controller - Interactive Launcher")
    print("=" * 60 + "\n")


def select_sensor_mode():
    """Interactive sensor mode selection."""
    print("센서 모드를 선택하세요:")
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


def main():
    """Main function."""
    print_banner()
    
    # Select sensor mode
    use_real_sensor, use_fake_proximity_publisher, mode_name = select_sensor_mode()
    print(f"\n선택된 모드: {mode_name}")
    
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
    
    print("\n" + "=" * 60)
    print("설정 요약:")
    print("=" * 60)
    print(f"  센서 모드: {mode_name}")
    if not use_real_sensor:
        print(f"  장애물 시뮬레이션: {'활성화' if obstacle_sim_enabled else '비활성화'}")
    if use_real_sensor:
        print(f"  가짜 발행자 사용: {'예' if use_fake_proximity_publisher else '아니오'}")
        print(f"  개별 센서 토픽: {'예' if use_individual_proximity_sensors else '아니오'}")
        if use_individual_proximity_sensors:
            print(f"  센서 매핑: {proximity_sensor_mapping}")
    print("=" * 60)
    print("\n런치 파일을 실행합니다...\n")
    
    # Get package share directory
    try:
        apf_pkg = get_package_share_directory("rbpodo_apf_controller")
        launch_file = os.path.join(apf_pkg, "launch", "apf_demo.launch.py")
    except Exception as e:
        print(f"패키지를 찾을 수 없습니다: {e}")
        print("먼저 워크스페이스를 빌드하고 source 해주세요:")
        print("  colcon build --packages-select rbpodo_apf_controller")
        print("  source install/setup.bash")
        sys.exit(1)
    
    # Build launch command
    cmd = [
        "ros2",
        "launch",
        "rbpodo_apf_controller",
        "apf_demo.launch.py",
        f"use_real_sensor:={'true' if use_real_sensor else 'false'}",
        f"use_fake_proximity_publisher:={'true' if use_fake_proximity_publisher else 'false'}",
        f"obstacle_sim_enabled:={'true' if obstacle_sim_enabled else 'false'}",
        f"use_individual_proximity_sensors:={'true' if use_individual_proximity_sensors else 'false'}",
        f"proximity_sensor_mapping:={proximity_sensor_mapping}",
    ]
    
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


