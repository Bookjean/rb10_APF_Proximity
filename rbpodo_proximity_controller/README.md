# rbpodo_proximity_controller

RB10-1300E용 근접센서 기반 APF 장애물 회피 컨트롤러 + Waypoint P2P 이동 패키지

## 개요

- **근접센서**(ecan_driver)의 거리값을 이용한 APF(Artificial Potential Field) 장애물 회피
- **Teach 기능**: 현재 로봇 관절 위치를 waypoint로 저장/로드 (YAML 파일)
- **P2P 순차 이동**: 저장된 여러 waypoint를 순서대로 이동하면서 장애물 회피
- 시뮬레이션(fake hardware)과 실제 로봇 모두 지원

## 패키지 구조

```
rbpodo_proximity_controller/
├── config/
│   ├── controller_params.yaml      # 컨트롤러/센서 파라미터
│   └── waypoints_example.yaml      # waypoint 예시
├── launch/
│   ├── proximity_demo.launch.py    # 시뮬레이션 데모 (fake hardware)
│   └── proximity_controller.launch.py  # 실제 로봇
├── rbpodo_proximity_controller/
│   ├── proximity_controller_node.py  # 메인 컨트롤러 (APF + P2P)
│   ├── waypoint_manager.py           # waypoint teach/save/load
│   ├── kdl_helper.py                 # KDL FK/Jacobian 유틸리티
│   └── fake_proximity_publisher.py   # 시뮬레이션용 가짜 근접센서
```

## 노드 설명

### 1. proximity_controller

APF 장애물 회피 + waypoint 순차 P2P 이동 컨트롤러

**구독 토픽:**
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/joint_states` | sensor_msgs/JointState | 현재 관절 상태 |
| `/proximity_distance` | std_msgs/Float32MultiArray | 근접센서 거리 (mm) |
| `/waypoint_manager/waypoints` | std_msgs/Float64MultiArray | waypoint 목록 |

**발행 토픽:**
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/position_controllers/commands` | std_msgs/Float64MultiArray | 관절 위치 명령 |
| `/proximity_debug/markers` | visualization_msgs/MarkerArray | RViz 시각화 |

**서비스:**
| 서비스 | 타입 | 설명 |
|--------|------|------|
| `~/enable` | std_srvs/SetBool | 컨트롤러 활성화/비활성화 |
| `~/execute_waypoints` | std_srvs/Trigger | waypoint 순차 실행 시작 |
| `~/stop` | std_srvs/Trigger | 실행 중지 |

### 2. waypoint_manager

로봇 위치 teach 및 waypoint 관리

**서비스:**
| 서비스 | 타입 | 설명 |
|--------|------|------|
| `~/save_current_position` | std_srvs/Trigger | 현재 위치를 waypoint로 저장 |
| `~/clear_waypoints` | std_srvs/Trigger | 모든 waypoint 삭제 |
| `~/delete_last_waypoint` | std_srvs/Trigger | 마지막 waypoint 삭제 |
| `~/save_to_file` | std_srvs/Trigger | YAML 파일로 저장 |
| `~/load_from_file` | std_srvs/Trigger | YAML 파일에서 로드 |
| `~/list_waypoints` | std_srvs/Trigger | waypoint 목록 출력 |

### 3. fake_proximity_publisher

시뮬레이션용 가짜 근접센서 (ecan_driver와 동일한 토픽 발행)

**발행 토픽:**
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/proximity_distance` | std_msgs/Float32MultiArray | 거리값 (mm) |
| `/proximity_detection` | std_msgs/Int32 | 감지 상태 (0/1) |

## 사용 방법

### 빌드

```bash
cd ~/ros2_ws
colcon build --packages-select rbpodo_proximity_controller
source install/setup.bash
```

### 시뮬레이션 데모

```bash
ros2 launch rbpodo_proximity_controller proximity_demo.launch.py
```

장애물 시뮬레이션 활성화:
```bash
ros2 launch rbpodo_proximity_controller proximity_demo.launch.py obstacle_sim_enabled:=true
```

### 실제 로봇

```bash
ros2 launch rbpodo_proximity_controller proximity_controller.launch.py robot_ip:=10.0.2.7
```

### Waypoint 저장 (Teach)

```bash
# 1. 로봇을 원하는 위치로 이동시킨 후 현재 위치 저장
ros2 service call /waypoint_manager/save_current_position std_srvs/srv/Trigger

# 2. 다른 위치로 이동 후 또 저장 (반복)
ros2 service call /waypoint_manager/save_current_position std_srvs/srv/Trigger

# 3. 저장된 waypoint 확인
ros2 service call /waypoint_manager/list_waypoints std_srvs/srv/Trigger

# 4. YAML 파일로 저장 (다음에 자동 로드됨)
ros2 service call /waypoint_manager/save_to_file std_srvs/srv/Trigger
```

### Waypoint 실행 (P2P + APF 장애물 회피)

```bash
# waypoint 순차 실행 시작
ros2 service call /proximity_controller/execute_waypoints std_srvs/srv/Trigger

# 실행 중지
ros2 service call /proximity_controller/stop std_srvs/srv/Trigger

# 컨트롤러 비활성화
ros2 service call /proximity_controller/enable std_srvs/srv/SetBool "{data: false}"
```

### Waypoint 관리

```bash
# 마지막 waypoint 삭제
ros2 service call /waypoint_manager/delete_last_waypoint std_srvs/srv/Trigger

# 모든 waypoint 삭제
ros2 service call /waypoint_manager/clear_waypoints std_srvs/srv/Trigger

# 파일에서 다시 로드
ros2 service call /waypoint_manager/load_from_file std_srvs/srv/Trigger
```

## 주요 파라미터 (controller_params.yaml)

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `k_att` | 2.0 | Attractive force 게인 |
| `k_rep` | 5.0 | Repulsive force 게인 |
| `d0` | 0.15 | 근접센서 영향 거리 (m, 150mm) |
| `v_max` | 1.0 | 최대 관절 속도 (rad/s) |
| `waypoint_tolerance` | 0.02 | waypoint 도달 판정 허용 오차 (rad) |
| `goal_tolerance` | 0.005 | EE 목표 도달 허용 오차 (m) |
| `ema_alpha` | 0.3 | 센서 EMA 필터 계수 |

## 동작 원리

1. **waypoint_manager**로 위치를 teach하여 waypoint 저장
2. **execute_waypoints** 서비스 호출 시 순차 실행 시작
3. 각 waypoint에 대해:
   - FK로 목표 EE 위치 계산
   - **Attractive force**: EE → 목표 waypoint 방향 인력
   - **Repulsive force**: 근접센서 거리 < d0일 때 반발력 생성
   - APF 합력 → Jacobian 기반 관절 토크 → 속도 적분 → 위치 명령
4. waypoint_tolerance 이내 도달 시 다음 waypoint로 전환
5. 모든 waypoint 완료 시 자동 정지

## 센서 연동

- **실제 로봇**: ecan_driver 패키지의 `/proximity_distance` 토픽 사용
- **시뮬레이션**: `fake_proximity_publisher`가 동일 토픽 발행
- 근접센서는 link3에 N/S/E/W 4방향으로 ToF 센서와 동일 위치에 장착
