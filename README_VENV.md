# rb10_APF_Proximity 가상환경 설정

이 패키지를 위한 독립적인 Python 가상환경을 설정하는 방법입니다.

## 빠른 시작

```bash
cd ~/rb_ws/src/rb10_APF_Proximity
./setup_venv.sh
source apf_venv/bin/activate
```

## 수동 설정

### 1. 가상환경 생성

```bash
cd ~/rb_ws/src/rb10_APF_Proximity
python3 -m venv apf_venv
```

### 2. 가상환경 활성화

```bash
source apf_venv/bin/activate
```

### 3. 의존성 설치

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

### 4. ROS2 경로 설정 (자동으로 setup_venv.sh가 처리)

venv에서 ROS2 Python 패키지를 사용할 수 있도록 경로를 추가합니다:

```bash
# apf_venv/lib/python3.10/site-packages/ros2_paths.pth 파일 생성
echo "/opt/ros/humble/lib/python3.10/site-packages" > apf_venv/lib/python3.10/site-packages/ros2_paths.pth
echo "/opt/ros/humble/local/lib/python3.10/dist-packages" >> apf_venv/lib/python3.10/site-packages/ros2_paths.pth
echo "/usr/lib/python3/dist-packages" >> apf_venv/lib/python3.10/site-packages/ros2_paths.pth
```

## 사용 방법

### 가상환경 활성화

```bash
cd ~/rb_ws/src/rb10_APF_Proximity
source apf_venv/bin/activate
```

### 워크스페이스 빌드

```bash
cd ~/rb_ws
source install/setup.bash
colcon build --packages-select rbpodo_proximity_controller rbpodo_apf_controller
```

### 실행

```bash
# 가상환경 활성화 상태에서
ros2 launch rbpodo_proximity_controller proximity_demo.launch.py
```

## 설치된 패키지

- `numpy`: 수치 계산
- `PyKDL`: 로봇 운동학/동역학 라이브러리
- `PyYAML`: YAML 파일 파싱
- `setuptools`: 패키지 설치 도구

## 주의사항

- ROS2 패키지들(`rclpy`, `sensor_msgs` 등)은 시스템에 설치되어 있어야 합니다.
- 가상환경을 활성화한 상태에서도 ROS2 빌드가 정상적으로 작동합니다.
- 다른 프로젝트의 가상환경과 충돌을 방지하기 위해 독립적인 가상환경을 사용합니다.

## .bashrc에 alias 추가 (선택사항)

```bash
# ~/.bashrc에 추가
alias apf_venv='source ~/rb_ws/src/rb10_APF_Proximity/apf_venv/bin/activate'
```

사용:
```bash
apf_venv  # 가상환경 활성화
```

