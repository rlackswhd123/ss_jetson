# ss_depth

`ss_depth`는 RealSense D435 depth camera와 LiDAR/SLAM 결과를 하나의 OpenCV 대시보드에 표시하는 ROS2 Python 패키지이다.

현재 단계의 목표는 실제 주행 제어가 아니라, 수동 이동 구루마에서 센서 결과를 사람이 모니터로 확인하는 것이다.

## 현재 기능

- RealSense color image 표시
- aligned depth image 기반 가까운 장애물 bbox 표시
- bbox 내부 depth median 거리 표시
- 하단 ROI 5분할 기반 `GO`, `TURN_LEFT`, `TURN_RIGHT` 판단
- 3프레임 연속 판단 안정화
- `/map` OccupancyGrid 지도 표시
- TF `map -> base_link` 우선, `/pose` fallback 기반 구루마 위치와 방향 표시
- 고정 목적지와 직선 점선 경로 표시
- 지도 화면과 카메라 화면을 좌우로 합성

## 주요 토픽

기본 토픽은 `ss_depth/config.py`에서 관리한다.

```python
COLOR_TOPIC = "/camera/camera/color/image_raw"
DEPTH_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw"
MAP_TOPIC = "/map"
POSE_TOPIC = "/pose"
MAP_FRAME = "map"
BASE_FRAME = "base_link"
```

Jetson에서 실제 토픽명이 다르면 `config.py`를 먼저 수정한다.

## MacBook 개발 기준

MacBook에서는 주로 코드 작성과 문법 확인만 진행한다.

```bash
cd /Users/saeumsoft/Desktop/ss_robot
python3 -B -c 'import ast, pathlib; [ast.parse(path.read_text()) for path in pathlib.Path("ss_depth/ss_depth").glob("*.py")]; print("syntax ok")'
```

MacBook에 ROS2, `rclpy`, `cv_bridge`, RealSense 장비가 없으면 실제 실행 검증은 할 수 없다.

## Git으로 Jetson에 옮기는 방법

현재 작업 폴더가 Git 저장소에 포함되어 있는지 먼저 확인한다.

```bash
cd /Users/saeumsoft/Desktop/ss_robot
git status
```

만약 `not a git repository`가 나오면, `ss_depth`를 관리할 Git 저장소에 추가한 뒤 push해야 한다.

예시:

```bash
cd /Users/saeumsoft/Desktop/ss_robot/ss_depth
git init
git add .
git commit -m "Add ss_depth dashboard package"
git branch -M main
git remote add origin <repo-url>
git push -u origin main
```

이미 상위 저장소에서 관리한다면 해당 저장소 루트에서 commit/push한다.

```bash
git add ss_depth
git commit -m "Add ss_depth dashboard package"
git push
```

## Jetson ROS2 workspace 준비

Jetson에서 ROS2 workspace를 준비한다.

```bash
mkdir -p ~/ss_robot_ws/src
cd ~/ss_robot_ws/src
git clone <repo-url> ss_depth
```

상위 저장소를 clone하는 구조라면, clone 후 `ss_depth` 패키지가 `~/ss_robot_ws/src/ss_depth`에 위치하도록 맞춘다.

필요 패키지를 설치한다.

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-realsense2-camera \
  ros-jazzy-cv-bridge \
  ros-jazzy-slam-toolbox \
  ros-jazzy-tf2-ros \
  python3-numpy \
  python3-opencv
```

의존성을 확인하고 빌드한다.

```bash
cd ~/ss_robot_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select ss_depth
source install/setup.bash
```

## Jetson 실행 순서

한 번에 여러 터미널을 열어 실행하려면 아래 스크립트를 사용한다.

```bash
cd ~/ss_robot_ws/src/ss_depth
./scripts/start_dashboard_terminals.sh
```

열어둔 터미널과 실행 중인 노드를 한 번에 종료하려면 아래 스크립트를 사용한다.

```bash
cd ~/ss_robot_ws/src/ss_depth
./scripts/stop_dashboard_terminals.sh
```

workspace 경로나 SLAM 설정 파일 위치가 다르면 환경변수로 바꿀 수 있다.

```bash
WORKSPACE_DIR=~/ss_robot_ws \
SLAM_PARAMS_FILE=~/ss_robot_ws/config/lidar_only_slam.yaml \
./scripts/start_dashboard_terminals.sh
```

기본 odometry 실행 명령은 `ros2_laser_scan_matcher` 기준이다. 다른 odometry 노드를 쓸 때는 `ODOM_COMMAND`만 바꾼다.

```bash
ODOM_COMMAND='ros2 run <odometry-package> <odometry-node>' \
./scripts/start_dashboard_terminals.sh
```

아래는 문제가 생겼을 때 개별로 확인하기 위한 수동 실행 순서이다.

터미널 1에서 RealSense를 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ss_robot_ws/install/setup.bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

터미널 2에서 LiDAR를 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ss_robot_ws/install/setup.bash
ros2 launch sllidar_ros2 sllidar_a2m7_launch.py
```

터미널 3에서 LiDAR 장착 위치 TF를 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ss_robot_ws/install/setup.bash
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 \
  base_link laser
```

터미널 4에서 `odom -> base_link`를 발행하는 odometry 또는 scan matcher를 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ss_robot_ws/install/setup.bash
ros2 run <odometry-package> <odometry-node>
```

예를 들어 `ros2_laser_scan_matcher`를 사용한다면 아래처럼 실행한다.

```bash
ros2 run ros2_laser_scan_matcher laser_scan_matcher \
  --ros-args \
  -p publish_odom:=/odom \
  -p publish_tf:=true
```

`odom -> base_link`는 static transform으로 고정하면 안 된다. 고정하면 구루마가 움직이거나 회전해도 대시보드 위치와 방향이 변하지 않는다.

터미널 5에서 LiDAR/SLAM을 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ss_robot_ws/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/home/susoft/ss_robot_ws/config/lidar_only_slam.yaml
```

터미널 6에서 대시보드를 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ss_robot_ws/install/setup.bash
ros2 run ss_depth dashboard_node
```

또는 launch 파일로 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ss_robot_ws/install/setup.bash
ros2 launch ss_depth dashboard.launch.py
```

## 실행 전 확인

토픽이 실제로 떠 있는지 확인한다.

```bash
ros2 topic list
```

아래 토픽이 보여야 한다.

```text
/camera/camera/color/image_raw
/camera/camera/aligned_depth_to_color/image_raw
/map
/pose
```

구루마 위치와 방향은 TF `map -> base_link`를 우선 사용한다. TF가 아직 없으면 `/pose`를 fallback으로 사용한다.

```bash
ros2 topic info /pose
ros2 run tf2_ros tf2_echo map base_link
```

## 화면 확인 기준

- RealSense color 화면이 오른쪽에 표시된다.
- 카메라 앞 0.4m 이내 물체에 bbox와 `Obstacle 0.38m` 형식의 거리 텍스트가 표시된다.
- 가까운 장애물 위치에 따라 `GO`, `TURN_LEFT`, `TURN_RIGHT`가 표시된다.
- `/map`이 들어오면 왼쪽에 LiDAR 지도가 표시된다.
- 지도 위에 구루마 위치, 방향, 고정 목적지, 직선 점선 경로가 표시된다.

## 문제 확인

OpenCV 창이 뜨지 않으면 Jetson이 GUI 세션에서 실행 중인지 확인한다.

```bash
echo $DISPLAY
```

토픽명이 다르면 `ss_depth/config.py`의 토픽 상수를 수정한 뒤 다시 빌드한다.

```bash
cd ~/ss_robot_ws
colcon build --packages-select ss_depth
source install/setup.bash
```

기본 설정은 Jetson CPU 부하를 줄이기 위해 대시보드 갱신 주기를 약 12.5fps로 낮추고, depth 검출과 map 렌더를 2프레임마다 한 번씩만 다시 계산한다. 더 부드럽게 보이고 싶으면 `ss_depth/config.py`의 `DASHBOARD_TIMER_SEC`, `DEPTH_DETECT_INTERVAL_FRAMES`, `MAP_RENDER_INTERVAL_FRAMES`를 조정한다.
