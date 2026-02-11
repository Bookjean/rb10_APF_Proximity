# rbpodo_apf_controller

RB10-1300E 6DOF 로봇의 Artificial Potential Field(APF) 컨트롤러.
EE를 목표점으로 이동시키면서 link3에 부착된 4개 ToF 센서(N/S/E/W)로 장애물을 회피한다.

## 빌드

```bash
cd ~/ros2_ws
colcon build --packages-select rbpodo_apf_controller
source install/setup.bash
```

**의존성:** `python-orocos-kdl`, `rclpy`, `sensor_msgs`, `geometry_msgs`, `visualization_msgs`, `std_srvs`, `rbpodo_description`, `rbpodo_bringup`

---

## 실행

```bash
ros2 launch rbpodo_apf_controller apf_demo.launch.py
```

RViz가 자동으로 열리며 로봇이 q=0 자세로 표시된다.
ToF 센서 FOV 콘(청록 와이어프레임)이 항상 표시된다.

---

## 목표점 이동 (Trajectory)

### 목표점 발행

```bash
ros2 topic pub /apf_goal geometry_msgs/PointStamped "{header: {frame_id: link0}, point: {x: 0.4, y: 0.0, z: 0.8}}" --once
```

EE가 목표점(노란 구)을 향해 이동하며, 파란 화살표(F_att)가 인력 방향을 표시한다.

### 다른 위치로 이동

```bash
# 위로
ros2 topic pub /apf_goal geometry_msgs/PointStamped "{header: {frame_id: link0}, point: {x: 0.0, y: 0.0, z: 1.2}}" --once

# 옆으로
ros2 topic pub /apf_goal geometry_msgs/PointStamped "{header: {frame_id: link0}, point: {x: 0.3, y: 0.3, z: 0.6}}" --once
```

새 목표점을 발행하면 즉시 방향이 전환된다.

---

## APF 켜기/끄기

서비스를 사용하여 APF 제어를 토글할 수 있다. 꺼도 시각화(마커, FOV)는 유지된다.

```bash
# APF 끄기
ros2 service call /apf_controller/enable std_srvs/srv/SetBool "{data: false}"

# APF 켜기
ros2 service call /apf_controller/enable std_srvs/srv/SetBool "{data: true}"
```

---

## 장애물 생성/관리

### 방법 1: 좌표로 직접 추가 (권장)

```bash
ros2 topic pub /add_obstacle geometry_msgs/PointStamped "{header: {frame_id: link0}, point: {x: 0.3, y: 0.0, z: 0.8}}" --once
```

여러 번 실행하면 장애물이 누적 추가된다. 각 장애물은 빨간 구로 RViz에 표시된다.

```bash
# 예: 장애물 2개 추가
ros2 topic pub /add_obstacle geometry_msgs/PointStamped "{header: {frame_id: link0}, point: {x: 0.3, y: 0.0, z: 0.8}}" --once
ros2 topic pub /add_obstacle geometry_msgs/PointStamped "{header: {frame_id: link0}, point: {x: 0.2, y: 0.3, z: 0.6}}" --once
```

### 방법 2: RViz에서 클릭

1. RViz 상단 툴바에서 **Publish Point** 클릭
2. 3D 뷰에서 로봇 모델 위 원하는 위치 클릭
3. 해당 위치에 장애물 추가됨 (`/clicked_point` 토픽)

### 방법 3: 런치 시 장애물 미리 설정

```bash
ros2 launch rbpodo_apf_controller apf_demo.launch.py obstacle_sim_enabled:=true
```

기본 장애물 위치: `[0.3, 0.0, 0.8]` (apf_params.yaml에서 변경 가능)

### 장애물 전체 삭제

```bash
ros2 service call /fake_tof_publisher/clear_obstacles std_srvs/srv/Trigger
```

---

## RViz 시각화

| 마커 | 색상 | 의미 |
|------|------|------|
| 구 | 초록 | EE 현재 위치 |
| 구 | 노란 | 목표점 |
| 화살표 | 파란 | 인력 (F_att) |
| 화살표 | 빨간 | 반발력 (F_rep) |
| 구 (작은) | 청록 | ToF 센서 위치 |
| 와이어프레임 콘 | 청록 | ToF 센서 FOV |
| 구 | 빨간 | 장애물 |

---

## 파라미터 튜닝

`config/apf_params.yaml` 수정 후 재실행:

### APF 제어 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `k_att` | 2.0 | 인력 게인 (클수록 목표로 빠르게 이동) |
| `k_rep` | 5.0 | 반발력 게인 (클수록 장애물 회피 강하게) |
| `k_q` | 1.0 | 관절속도 게인 (tau -> qdot) |
| `v_max` | 1.0 | 최대 관절속도 (rad/s) |
| `d0` | 0.5 | 반발력 활성 거리 (m) |
| `goal_tolerance` | 0.005 | 목표 도달 판정 거리 (m) |
| `control_rate` | 100.0 | 제어 루프 주기 (Hz) |
| `dt` | 0.01 | 적분 시간 간격 (s) |
| `ema_alpha` | 0.3 | ToF EMA 필터 계수 (0~1, 클수록 빠른 반응) |

### EE / 센서 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `ee_offset` | [0, -0.1153, 0] | link6 로컬 프레임에서 EE 오프셋 (m) |
| `tof_fov` | 0.174533 | ToF 시야각 (rad, ~10 deg) |
| `tof_range` | 0.5 | ToF FOV 시각화 거리 (m) |

---

## 토픽/서비스 요약

### 토픽

| 토픽 | 타입 | 방향 | 설명 |
|------|------|------|------|
| `/joint_states` | JointState | 구독 | 현재 관절 각도 |
| `/apf_goal` | PointStamped | 구독 | 목표점 (frame_id: link0) |
| `/add_obstacle` | PointStamped | 구독 | 장애물 추가 (좌표 입력) |
| `/clicked_point` | PointStamped | 구독 | 장애물 추가 (RViz 클릭) |
| `/link3_tof_{N,S,E,W}/range` | Range | 구독 | ToF 센서 거리값 |
| `/position_controllers/commands` | Float64MultiArray | 발행 | 관절 위치 명령 |
| `/apf_debug/markers` | MarkerArray | 발행 | APF 디버그 시각화 |
| `/obstacle_markers` | MarkerArray | 발행 | 장애물 시각화 |

### 서비스

| 서비스 | 타입 | 설명 |
|--------|------|------|
| `/apf_controller/enable` | SetBool | APF 켜기/끄기 |
| `/fake_tof_publisher/clear_obstacles` | Trigger | 장애물 전체 삭제 |

---

## 패키지 구조

```
rbpodo_apf_controller/
├── config/
│   ├── apf_params.yaml          # 튜닝 파라미터
│   └── apf_rviz.rviz            # RViz 설정
├── launch/
│   └── apf_demo.launch.py       # 통합 런치
├── urdf/
│   ├── rb10_1300e_tof_sensors.urdf.xacro  # ToF 센서 프레임
│   └── rb10_1300e_apf.urdf.xacro          # 로봇 + 센서 래퍼
└── rbpodo_apf_controller/
    ├── apf_node.py               # APF 컨트롤러 노드
    ├── kdl_helper.py             # KDL FK/Jacobian 유틸
    ├── fake_tof_publisher.py     # 시뮬레이션 ToF 퍼블리셔
    ├── go_home.py                # 홈 이동 (장애물 제거 + APF 끄기)
    └── go_home_keep_obs.py       # 홈 이동 (장애물 유지 + APF 끄기)
```

---

## 홈 위치 이동

```bash
# 장애물 제거 + APF 끄기 + 홈(all zero) 이동
ros2 run rbpodo_apf_controller go_home

# 장애물 유지 + APF 끄기 + 홈(all zero) 이동
ros2 run rbpodo_apf_controller go_home_keep_obs
```

| 명령 | APF | 장애물 | 홈 이동 |
|------|-----|--------|---------|
| `go_home` | 끄기 | 제거 | O |
| `go_home_keep_obs` | 끄기 | 유지 | O |

---

## 빠른 시작 예제

```bash
# 1. 런치
ros2 launch rbpodo_apf_controller apf_demo.launch.py

# 2. 장애물 배치
ros2 topic pub /add_obstacle geometry_msgs/PointStamped "{header: {frame_id: link0}, point: {x: 0.3, y: 0.0, z: 0.8}}" --once

# 3. 목표점으로 이동 (장애물 회피하면서)
ros2 topic pub /apf_goal geometry_msgs/PointStamped "{header: {frame_id: link0}, point: {x: 0.4, y: 0.0, z: 0.8}}" --once

# 4. APF 일시정지
ros2 service call /apf_controller/enable std_srvs/srv/SetBool "{data: false}"

# 5. APF 재개
ros2 service call /apf_controller/enable std_srvs/srv/SetBool "{data: true}"

# 6. 장애물 초기화
ros2 service call /fake_tof_publisher/clear_obstacles std_srvs/srv/Trigger
```
