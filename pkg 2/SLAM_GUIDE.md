# aircore_description — SLAM 완성 가이드 (ROS 2 Jazzy + Gazebo Harmonic)

## 1. 발견한 문제 & 고친 것

이 패키지는 `package.xml`/`CMakeLists.txt`만 ROS 1(catkin) → ROS 2(ament_cmake)로
절반쯤 바뀌어 있었고, urdf/launch는 그대로 ROS 1 + **Gazebo Classic**
(`libgazebo_ros_diff_drive.so`, `libgazebo_ros_ray_sensor.so`, `.launch` XML,
`ros_control` yaml) 상태였습니다. 게다가 `package.xml`의 `<package format3>`
는 XML 문법 자체가 깨져 있어서 **`colcon build`가 아예 실패**하는 상태였습니다
(직접 빌드해서 확인함).

Ubuntu 24.04 + ROS 2 Jazzy 조합에서는 Gazebo Classic이 공식 지원되지 않고
**Gazebo Harmonic(gz-sim)** 이 기본입니다. 그래서 이번에 다음을 전부
Harmonic 기준으로 다시 맞췄습니다.

| 파일 | 변경 내용 |
|---|---|
| `package.xml` | XML 문법 수정 + `ros_gz_sim`, `ros_gz_bridge`, `slam_toolbox`, `rviz2`, `joint_state_publisher_gui` 의존성 추가 |
| `CMakeLists.txt` | `config/` 디렉터리도 설치되도록 추가 |
| `urdf/aircore.gazebo` | 라이다: `ray` → `gpu_lidar` 센서로 변경. 바퀴 구동: `libgazebo_ros_diff_drive.so` → `gz::sim::systems::DiffDrive` 플러그인으로 교체 (바퀴 간격도 실제 조인트 좌표 기준 0.1505m로 보정, 기존 값 0.2는 오차 있었음). `JointStatePublisher` 플러그인 추가 |
| `worlds/aircore_world.sdf` | 새로 생성. 바닥/조명 + gz-sim 필수 시스템 플러그인(Physics/Sensors/SceneBroadcaster/UserCommands) + 회피 테스트용 박스 1개 |
| `config/bridge.yaml` | 새로 생성. gz ↔ ROS2 토픽 브릿지 설정 (`/cmd_vel`, `/odom`, `/tf`, `/scan`, `/joint_states`, `/clock`) |
| `config/mapper_params_online_async.yaml` | 새로 생성. slam_toolbox 온라인 매핑 파라미터 |
| `config/aircore.rviz` | 새로 생성. RViz2 기본 설정 |
| `launch/*.launch` (ROS1 XML) | 삭제하고 `display.launch.py` / `gazebo.launch.py` / `slam.launch.py` 로 재작성 |
| `launch/controller.launch`, `controller.yaml` | 삭제. `ros_control` 방식 velocity controller는 더 이상 필요 없음 — gz-sim `DiffDrive` 플러그인이 `/cmd_vel`을 직접 받아 바퀴를 구동함 |

## 2. 아키텍처 한눈에 보기

```
[cmd_vel(ROS2)] --bridge--> [DiffDrive 플러그인] --> 바퀴 조인트 구동
                                    |
                                    v
[gz odom/tf] --bridge--> [/odom, /tf(ROS2)]
[gpu_lidar]  --bridge--> [/scan(ROS2)] --> slam_toolbox --> [/map, map→odom tf]
```

- 시뮬레이터: Gazebo Harmonic (`gz sim`)
- ROS2 ↔ Gazebo 통신: `ros_gz_bridge` (설정 파일 기반, `config/bridge.yaml`)
- SLAM: `slam_toolbox`의 `async_slam_toolbox_node` (online async 모드)

## 3. 사전 설치

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-rviz2 \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-slam-toolbox \
  ros-jazzy-teleop-twist-keyboard
```

## 4. 빌드

```bash
cd /mnt/c/Users/admin/aircore
source /opt/ros/jazzy/setup.bash
colcon build --packages-select aircore_description
source install/setup.bash
```

## 5. 실행 순서 (터미널 4개)

**터미널 1 — Gazebo 시뮬레이션 + 로봇 스폰 + gz↔ROS2 브릿지**
```bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/admin/aircore/install/setup.bash
ros2 launch aircore_description gazebo.launch.py
```
Gazebo Harmonic 창이 뜨고 로봇이 스폰됩니다. 좌측 하단 재생(▶) 버튼이 눌려있는지
확인하세요(월드에 `-r` 옵션을 넣어놔서 자동 재생되긴 합니다).

확인:
```bash
ros2 topic list          # /scan, /odom, /tf, /cmd_vel, /joint_states 보이는지
ros2 topic hz /scan       # 약 10Hz로 발행되는지
```

**터미널 2 — RViz2로 시각화**
```bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/admin/aircore/install/setup.bash
rviz2 -d install/aircore_description/share/aircore_description/config/aircore.rviz
```
왼쪽 Displays 패널에서 `Fixed Frame`을 `odom` → (slam 실행 후) `map` 으로
바꿔가며 확인하면 됩니다.

**터미널 3 — slam_toolbox 실행**
```bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/admin/aircore/install/setup.bash
ros2 launch aircore_description slam.launch.py
```
RViz2에서 `Fixed Frame`을 `map`으로 바꾸고 `Map` 디스플레이를 켜면
지도가 그려지기 시작합니다.

**터미널 4 — 로봇을 움직여서 맵 채우기**
```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
`i/,/j/l/k` 키로 이동시키면서 월드 곳곳을 스캔하세요. (박스 장애물이
`worlds/aircore_world.sdf`에 하나 있으니 그 주변도 지나가 보세요.)

## 6. 맵 저장 (Day3 완료 기준)

```bash
ros2 run nav2_map_server map_saver_cli -f ~/aircore_map
```
`nav2_map_server`가 아직 없다면 `sudo apt install ros-jazzy-nav2-map-server`로
설치하거나, slam_toolbox 자체 서비스로 저장해도 됩니다:
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'aircore_map'}}"
```

## 7. 자주 막히는 부분

- **RViz2에 아무것도 안 보임**: `Fixed Frame`이 존재하지 않는 프레임으로 설정돼
  있을 때 흔합니다. `gazebo.launch.py`만 켠 상태면 `odom`, slam까지 켠 상태면
  `map`으로 맞추세요.
- **`/scan`이 안 뜸**: 터미널 1에서 에러 로그 확인. WSL2에서는 GPU 렌더링
  (ogre2)이 필요한 `gpu_lidar` 센서 특성상 소프트웨어 렌더링으로 느려질 수
  있습니다. 그럴 땐 `export LIBGL_ALWAYS_SOFTWARE=1` 후 재실행해보세요.
- **로봇이 안 움직임**: `ros2 topic echo /cmd_vel`로 teleop이 실제로 퍼블리시하는지,
  `ros2 topic info /cmd_vel`로 DiffDrive 플러그인이 구독 중인지 확인하세요.
- **바퀴가 미끄러짐/이상하게 돔**: `urdf/aircore.gazebo`의 `wheel_separation`,
  `wheel_radius` 값이 실측과 다르면 오도메트리가 어긋납니다. 실제 바퀴 지름/축간
  거리를 재서 필요하면 수정하세요.

## 8. 이번 단계에서 일부러 안 한 것

- Nav2, costmap, 장애물 회피, STM32 UART 스펙 문서 — 계획표의 Day4 이후 항목이라
  이번 요청 범위(Day1~3, SLAM 완성)에서 제외했습니다.