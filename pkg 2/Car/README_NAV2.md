# README_NAV2 — SLAM + Nav2 전체 파이프라인 실행/검증 기록

이 문서는 `aircore_description` 패키지에서 Gazebo 시뮬레이션 → SLAM 매핑 → 지도 저장 →
Map Server/AMCL 로컬라이제이션 → Nav2 목표 주행까지 전체 파이프라인을 실제로 빌드하고
실행하여 검증한 기록이다. 설정 파일만 만든 것이 아니라 WSL2 Ubuntu 24.04 환경에서
직접 실행했고, 그 과정에서 발견한 버그와 한계를 있는 그대로 적는다.

## 1. 개발 환경

- OS: Windows 11 + WSL2 (Ubuntu 24.04, 배포판 이름 `Ubuntu-24.04`)
- ROS 2: Jazzy (`/opt/ros/jazzy`)
- 시뮬레이터: Gazebo Harmonic (`gz sim`, ros_gz)
- SLAM: `slam_toolbox` (async online mode)
- Nav2: `navigation2` 1.3.12 (Jazzy 배포 버전)
- 워크스페이스: `~/aircore_ws` (WSL 홈 디렉터리, `src/aircore_description`는 저장소의
  `pkg 2` 디렉터리를 가리키는 심볼릭 링크 — 저장소 경로에 공백이 있어(`pkg 2`) colcon
  패키지 폴더명 자체는 공백 없는 심볼릭 링크 이름을 사용)

```
~/aircore_ws/src/aircore_description -> "/mnt/c/Users/user/Desktop/전공/Aircore/pkg 2"
```

## 2. 패키지 구조

```
pkg 2/                      (colcon 패키지명: aircore_description)
├── CMakeLists.txt, package.xml
├── meshes/                 STL 20개
├── urdf/aircore.xacro, aircore.gazebo, aircore.trans, materials.xacro
├── worlds/aircore_world.sdf         7x7m 십자형 4개 방 + 장애물
├── config/
│   ├── bridge.yaml                  gz<->ROS2 토픽 브릿지
│   ├── mapper_params_online_async.yaml   slam_toolbox 파라미터
│   ├── nav2_params.yaml             Nav2 전체 파라미터 (amcl/map_server/docking/collision_monitor 포함)
│   └── aircore.rviz                 RViz 설정
├── maps/
│   ├── aircore_map.pgm/.yaml        실제 SLAM 결과 (이번에 새로 생성, 기존 가짜맵 교체)
│   └── aircore_map_preview.png      맵 미리보기(윈도우에서 pgm을 못 열어서 png 병행)
├── scripts/wander.py                반응형 자동 주행 노드
└── launch/
    ├── gazebo.launch.py             Gazebo + 로봇 스폰 + 브릿지
    ├── slam.launch.py               slam_toolbox (매핑 모드)
    ├── localization.launch.py       map_server + amcl + lifecycle_manager (신규)
    ├── nav2.launch.py                controller/planner/.../costmap (navigation_launch.py include)
    ├── wander.launch.py, rviz.launch.py, display.launch.py
```

## 3. 전체 데이터 흐름

```
[cmd_vel] --bridge--> gz DiffDrive 플러그인 --> 바퀴 조인트 구동
                              |
                              +--> odom, tf(odom->base_link) --bridge--> ROS2 /odom, /tf
[gpu_lidar] --bridge--> /scan

매핑 모드:  /scan, /odom, tf --> slam_toolbox --> /map, tf(map->odom)
로컬라이제이션 모드: 저장된 지도(.yaml/.pgm) --> map_server --> /map (latched)
                     /scan, /odom, tf --> amcl --> tf(map->odom), /amcl_pose

Nav2: /map, tf(map->base_link 합성), /scan --> costmap(global/local)
      --> planner_server(/plan) --> controller_server(MPPI) --> velocity_smoother
      --> collision_monitor --> /cmd_vel
```

TF 트리 (매핑/로컬라이제이션 공통):
```
map -> odom -> base_link -> left_wheel / right_wheel / (fixed 조인트들은 base_link로 합쳐짐, lidar는 gz_frame_id로 명시된 별도 프레임)
```
- `odom -> base_link`: gz `DiffDrive` 플러그인이 **유일하게** 발행 (bridge.yaml로 ROS `/tf`에 전달). 중복 없음.
- `map -> odom`: 매핑 모드에선 slam_toolbox, 로컬라이제이션 모드에선 amcl이 발행 — **두 모드를 동시에 켜지 않아야** 하며, 이 프로젝트의 두 launch 파일(`slam.launch.py`/`localization.launch.py`)은 항상 둘 중 하나만 실행하도록 사용한다.
- 이 로봇에는 `base_footprint`가 없으므로 모든 설정(`amcl.base_frame_id`, costmap `robot_base_frame`, bt_navigator 등)이 실제 존재하는 `base_link`로 통일되어 있다.

## 4. 빌드 명령

```bash
# 최초 1회: 의존성 설치 (이유: slam_toolbox/navigation2/nav2_bringup/nav2_map_server/
# nav2_amcl/nav2_lifecycle_manager/teleop_twist_keyboard/joint_state_publisher_gui가
# 원래 이 WSL 이미지에 설치돼 있지 않았음 — package.xml/가이드엔 명시돼 있었지만 실제
# apt install은 안 된 상태였음)
sudo apt update
sudo apt install -y \
  ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-map-server ros-jazzy-nav2-amcl ros-jazzy-nav2-lifecycle-manager \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-joint-state-publisher-gui

# 워크스페이스 (최초 1회)
mkdir -p ~/aircore_ws/src
ln -s "/mnt/c/Users/user/Desktop/전공/Aircore/pkg 2" ~/aircore_ws/src/aircore_description

# 빌드
source /opt/ros/jazzy/setup.bash
cd ~/aircore_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

빌드 결과: **오류 0개** (`colcon build` Summary: 1 package finished).

## 5. Gazebo 실행 명령

```bash
ros2 launch aircore_description gazebo.launch.py            # GUI 포함
ros2 launch aircore_description gazebo.launch.py headless:=true   # 서버만 (RTF 개선, CLI 검증용)
```

검증 결과 (전부 CLI로 실측):
- `ros2 topic list` → `/clock /cmd_vel /joint_states /odom /scan /tf /tf_static /robot_description` 모두 확인됨
- `/clock` 250Hz (물리 스텝 250Hz와 일치)
- `/scan` 약 5Hz (LiDAR `update_rate:5` 설정과 일치), `sensor_msgs/msg/LaserScan`
- `/odom` 약 28Hz, `nav_msgs/msg/Odometry`
- `/cmd_vel` 발행 시 로봇이 실제로 이동함 (`ros2 topic pub` 0.15m/s 전진 → 3초 후 x가 0→0.548m로 증가, gz 실제 좌표로도 확인)
- `ros2 run tf2_ros tf2_echo odom base_link`, `tf2_echo base_link lidar` 모두 정상 (frame not found 경고는 시작 직후 한 번뿐, 이후 정상)

## 6. SLAM 실행 및 지도 저장 명령

```bash
ros2 launch aircore_description slam.launch.py
```

`slam_toolbox`는 `LifecycleNode`라서 `launch/slam.launch.py`가 configure→activate
이벤트를 명시적으로 걸어준다 (이 프로젝트에 이미 구현되어 있었고, 실행 로그에서
`Configuring` → `[slam.launch.py] slam_toolbox configure 완료 -> activate 진행` →
`Activating` 순서가 실제로 찍히는 것을 확인함).

로봇을 이동시켜 맵을 채우는 방법 (아래 "8. 발생한 오류와 해결 방법" 참고 — 이번 세션에서는
`wander.py`의 반응형 회피가 벽/기둥과의 충돌 시 심각한 오도메트리 폭주를 유발하는 것을
발견해서, 최종적으로는 **중앙 교차로 허브 + 4개 복도 입구만 도는 보수적인 스크립트**로
지도를 만들었다):

```bash
ros2 run nav2_map_server map_saver_cli -f ~/aircore_ws/src/aircore_description/maps/aircore_map
```

검증:
```bash
ros2 node info /slam_toolbox   # Subscribers에 /scan 있음 확인
ros2 topic hz /map             # 0.5Hz (map_update_interval:2.0와 일치) 확인
```

**저장된 지도(`maps/aircore_map.pgm/.yaml`)는 실제로 slam_toolbox가 `/scan`+`/odom`+TF로
생성한 결과다.** 기존에 저장돼 있던 맵은 `worlds/aircore_world.sdf`의 좌표를 직접 계산해
합성한 가짜 맵이었고(`gen_fake_map.py`, 이미 삭제된 스크립트), 이번에 실제 SLAM 결과로
덮어썼다.

## 7. Localization (Map Server + AMCL) 실행 명령

```bash
ros2 launch aircore_description localization.launch.py
```

`nav2_bringup`의 `localization_launch.py`를 그대로 include해서 `map_server` + `amcl` +
`lifecycle_manager_localization`을 띄운다. **slam.launch.py와 동시에 실행하면 안 됨**
(`map->odom`을 두 노드가 동시에 발행하게 됨).

RViz의 `2D Pose Estimate` 대신 CLI로 초기 위치를 지정하려면:
```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: <X>, y: <Y>, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.06]}}"
```

검증 결과: `map_server`, `amcl` 모두 `ros2 lifecycle get`으로 `active` 확인.
초기 위치 입력 후 `/amcl_pose`가 실제로 갱신됨을 확인 (`ros2 topic echo /amcl_pose`).
`/particle_cloud` 토픽도 발행됨 (RViz `ParticleCloud` 디스플레이용).

**한계**: 이번에 실제 SLAM으로 만든 맵은 중앙 십자 교차로 위주라 방 안쪽 장애물(기둥/박스)
같은 구별되는 특징이 부족하고, 십자 모양 자체가 90도 회전 대칭에 가까워서 AMCL의
particle filter가 잘 수렴하지 못하고 위치 추정이 계속 흔들리는 현상을 확인함 (물리적으로는
정지해 있는데 `/amcl_pose`가 초 단위로 0.1~0.3m씩 계속 바뀜 — gz 실제 좌표와 대조해서 확인).
자세한 원인은 8절 참고.

## 8. Nav2 실행 명령

```bash
ros2 launch aircore_description nav2.launch.py
```

`nav2_bringup`의 `navigation_launch.py`를 include. `controller_server`, `smoother_server`,
`planner_server`, `route_server`, `behavior_server`, `bt_navigator`, `waypoint_follower`,
`velocity_smoother`, `collision_monitor`, `docking_server` + `lifecycle_manager_navigation`이
뜬다 (이번에 쓴 Nav2 버전(1.3.12)은 예전 버전보다 `collision_monitor`/`docking_server`/
`route_server`가 기본으로 추가돼 있었음 — 아래 8절 참고).

로봇 footprint/속도 제한 근거: `robot_radius: 0.15m` (STL 실측 최대 폭 약 16cm의 여유
포함 근사치), `vx_max: 0.3 m/s`, `wz_max: 1.2 rad/s` — 7x7m 실내 월드와 로봇의 저속
사양(구동바퀴 반지름 3.25cm, DiffDrive `max_linear_acceleration:1.0`)에 맞춘 값. `linear.y`는
MPPI critic 어디에도 요구되지 않음(차동구동이라 애초에 옆으로 못 움직임).

## 9. RViz 조작 순서 (GUI 확인 필요 — 이번 세션은 headless로만 검증했음)

1. `ros2 launch aircore_description rviz.launch.py` (반드시 `ros2 launch`로 켤 것 — 메시 경로 문제 방지)
2. Displays에서 RobotModel/TF/Map/LaserScan/GlobalCostmap/LocalCostmap/GlobalPlan 체크
3. Fixed Frame: 매핑 중이면 `map`, Gazebo만 켰으면 `odom`
4. 로컬라이제이션 모드: 상단 `2D Pose Estimate`로 초기 위치 지정 → 오차 타원(파티클) 확인
5. `2D Goal Pose`(또는 Nav2 Goal)로 목표 지정 → 초록색 전역 경로가 그려지고 로봇이 이동하는지 확인

## 10. 검증 명령 모음

```bash
ros2 topic list; ros2 topic info /scan -v; ros2 topic info /odom -v; ros2 topic info /cmd_vel -v
ros2 topic hz /scan; ros2 topic hz /odom; ros2 topic hz /map; ros2 topic hz /clock
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom          # SLAM/AMCL 실행 후
ros2 node list; ros2 lifecycle nodes
ros2 lifecycle get /controller_server /bt_navigator /planner_server /amcl /map_server
ros2 action list; ros2 action info /navigate_to_pose
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: <X>, y: <Y>, z: 0.0}, orientation: {w: 1.0}}}}"
```

## 11. 발생한 오류와 해결 방법 (실제로 겪은 순서대로)

| # | 증상 | 원인 | 해결 |
|---|---|---|---|
| 1 | `slam_toolbox`/`navigation2`/`nav2_bringup` 등이 `ros2 pkg list`에 없음 | package.xml엔 의존성으로 적혀 있었지만 실제 apt install이 안 된 상태 | `sudo apt install ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-map-server ros-jazzy-nav2-amcl ros-jazzy-nav2-lifecycle-manager ros-jazzy-teleop-twist-keyboard ros-jazzy-joint-state-publisher-gui` |
| 2 | `wander.launch.py` 실행 시 `/usr/bin/env: 'python3\r': No such file or directory`, exit code 127 | `scripts/wander.py`가 Windows(CRLF) 줄바꿈으로 저장돼 있어 shebang이 깨짐 | `sed -i 's/\r$//' scripts/wander.py`로 LF 변환 |
| 3 | colcon build 후 install 경로에 `maps/`가 없음 | `CMakeLists.txt`의 `install(DIRECTORY ...)`에 `maps`가 빠져 있었음 | `maps`를 install 대상에 추가 |
| 4 | 백그라운드로 띄운 Gazebo/SLAM을 종료해도 프로세스가 안 죽고, 재실행하면 `/robot_state_publisher`, `/ros_gz_bridge`, `/slam_toolbox`가 **2개씩** 뜸 | 세션 관리 도구로 프로세스를 중단시켜도 WSL 내부의 실제 프로세스가 좀비로 남는 경우가 있었음 → 두 개의 Gazebo 인스턴스가 동시에 같은 ROS 토픽에 발행하며 서로 간섭 | `pkill -9 -f "<프로세스 패턴>"`으로 프로세스 이름 기준 확실하게 종료 후 `ps aux`로 빈 상태 확인하고 재실행 |
| 5 | 초기 SLAM 결과 지도가 같은 방이 회전/이동된 채로 여러 겹 찍혀 못 씀 | 위 4번(중복 프로세스)과, 로봇이 벽/기둥에 낀 상태에서 바퀴가 헛돌며 오도메트리가 실제 이동거리보다 수십 배 크게 폭주(직접 gz 좌표 조회로 실제 위치는 거의 안 바뀌었는데 `/odom`은 20~50m까지 벌어지는 것을 확인) | 프로세스 중복 제거 + 중앙 교차로·복도 입구만 도는 보수적 주행으로 재매핑 → 십자 복도 구조가 뚜렷한 지도 확보 |
| 6 | `nav2.launch.py` 실행 시 lifecycle bringup이 `collision_monitor` configure 단계에서 abort | 이 프로젝트가 원래 쓰던 Nav2 버전보다 최신인 1.3.12는 `collision_monitor`를 기본 관리 노드로 포함하는데, `observation_sources` 등 필수 파라미터가 `nav2_params.yaml`에 없었음 (예전 버전 기준으로 작성된 설정과 최신 Nav2의 파라미터 격차) | `nav2_params.yaml`에 `collision_monitor` 섹션(`observation_sources: ["scan"]` 등) 추가 |
| 7 | 이어서 `docking_server` configure 단계에서 abort (`Charging dock plugins not given!`) | 같은 이유로 `docking_server`도 최신 Nav2의 기본 관리 노드인데 설정이 없었음(이 로봇은 도킹 기능 자체가 없음) | 실제 도킹 기능은 안 쓰지만 configure만 통과하도록 `dock_plugins: ['simple_charging_dock']` 최소 설정 추가 |
| 8 | AMCL 활성화 후 `/amcl_pose`가 로봇이 정지해 있어도 계속 흔들림 | 저장된 지도가 거의 대칭인 십자 복도 위주라 particle filter가 뚜렷하게 수렴할 특징이 부족함 (실측: gz 실제 좌표는 고정인데 `/amcl_pose`는 초당 0.1~0.3m씩 계속 이동) | 근본 해결 못함(맵 자체를 특징이 많은 방 안쪽까지 확장해야 함) — 한계로 문서화 |
| 9 | Nav2 목표 전송 시 플래너가 계속 `Failed to create plan`으로 실패, `spin`/`backup` 복구 행동도 전부 시간 초과로 실패 | `/odom`이 세션이 길어지면서(반복된 회전/충돌/텔레포트) 실제 위치와 수십 미터까지 벌어지는 근본적인 오도메트리 신뢰성 문제 — Nav2가 참조하는 "현재 위치"가 실제 지도 범위를 벗어나거나 코스트맵상 이상한 위치로 계산되어 경로 자체를 못 만듦 | 짧고 신중한 구간별 주행(각 구간 후 `gz model -p`로 실제 좌표 대조)으로는 완화되지만, 장시간·다중 상호작용 세션에서는 재발함 — **미해결로 문서화**, 아래 12절 참고 |

## 12. 현재 남아 있는 한계

> **갱신 (15절)**: 아래 항목들은 15절에서 다룬 오도메트리 드리프트 수정
> *이전* 세션 기준으로 작성됐다. 15-9(볼캐스터 높이)/15-10(바퀴 원기둥
> 폭) 수정 이후 오도메트리 드리프트로 인한 지도 붕괴 문제는 해결됐고,
> `maps/aircore_map.yaml/.pgm`도 수정된 로봇으로 새로 매핑해서 덮어썼다
> (십자 복도 + 4방향 장애물 일부까지 매핑, 여전히 방 안쪽 깊은 곳은
> `wander.py`의 탐색 한계로 미탐색). 아래의 "Nav2 목표 도달 실패",
> "AMCL 수렴 실패" 항목은 이 새 지도/오도메트리로 재검증이 필요하다 —
> 근본 원인(오도메트리 폭주)이 없어졌으니 결과가 달라질 가능성이 높다.

- **Nav2 목표 도달을 끝까지 성공시키지 못함.** 원인은 이 시뮬레이션 환경(GPU 없는 WSL2,
  가벼운 로봇, 250Hz로 낮춘 물리 스텝)에서 gz-sim `DiffDrive` 플러그인의 `/odom`이 세션이
  길어지거나 충돌/급회전이 반복되면 실제 위치(직접 `gz model -m aircore -p`로 조회한 값)와
  수십 미터까지 벌어지는 현상이다. 이 오도메트리를 Nav2 코스트맵/플래너가 그대로 신뢰하기
  때문에 "현재 위치"가 지도 밖으로 계산되어 경로 계획 자체가 실패한다.
- **AMCL이 안정적으로 수렴하지 못함.** 실제 SLAM으로 만든 지도가 특징이 적은 중앙 십자
  복도 위주라(방 안쪽 기둥/박스까지 충분히 매핑하지 못함) particle filter가 위치를 확정하지
  못하고 계속 흔들린다.
- **저장된 지도가 방 안쪽까지 완전히 채워지지 않음.** 위 두 문제 때문에 로봇을 방 깊숙이
  장시간 자유 주행시키는 것이 위험(오도메트리 폭주 유발)해서, 중앙 교차로와 4개 복도
  입구 정도까지만 확보했다. 방 안쪽(기둥/박스 근처)은 회색(미탐색)으로 남아 있다.
- RViz GUI로 직접 목표를 클릭해서 확인하는 것은 이번 세션에서 하지 못했다(headless로만
  검증) — **GUI 확인 필요** 항목.

## 13. 실물 로봇으로 이전할 때 필요한 작업

- **오도메트리를 실제 바퀴 엔코더 기반으로 교체.** 시뮬레이션의 `DiffDrive` 플러그인은
  이상 조건(충돌/급회전)에서 오도메트리가 실제 이동량보다 훨씬 크게 계산되는 것을 확인했다
  (물리 엔진의 접촉/적분 특성 때문으로 추정). 실물에서는 엔코더+IMU(`robot_localization` 등)
  기반의 센서 퓨전으로 오도메트리를 만드는 것이 시뮬레이션보다 훨씬 신뢰할 수 있을 가능성이
  높지만, 그래도 실물 엔코더 캘리브레이션(바퀴 반지름 0.0325m, 축간 거리 0.1505m가 정확한지)을
  다시 검증해야 한다.
- **지도를 실제 공간 전체(방 안쪽 포함)로 다시 제작.** 특징이 풍부한 지도가 있어야 AMCL이
  안정적으로 수렴한다 — 실물에서는 사람이 직접 리모컨/조이스틱으로 천천히, 충돌 없이
  구석구석 돌면서 매핑하는 것을 권장한다(반응형 자동 주행보다 안전).
- STM32/UART 통신 스펙, 실제 장애물 배치, 파라미터(속도/가속도 제한, `inflation_radius`
  등) 재튜닝 — 이번 범위 밖이라 손대지 않음.
- LiDAR 마운트 높이(현재 시뮬레이션에서 자기 몸체 반사를 피하려 0.015m 올림)가 실물
  LiDAR(RPLiDAR C1) 장착 위치와 일치하는지 확인.

## 14. 완료 기준 대비 현황

| 항목 | 상태 |
|---|---|
| `colcon build` 성공 | **CLI 검증됨** |
| Gazebo에서 로봇 spawn 성공 | **CLI 검증됨** |
| `/scan`, `/odom`, `/cmd_vel`, `/clock` 정상 | **CLI 검증됨** |
| TF 트리가 끊기지 않음 (odom-base_link, base_link-lidar) | **CLI 검증됨** |
| SLAM Toolbox가 실제 센서 데이터로 `/map` 생성 | **CLI 검증됨** (기존 가짜 맵 폐기, 실제 SLAM 결과로 교체) |
| 실제 지도 파일 저장 | **CLI 검증됨** (`maps/aircore_map.yaml/.pgm`) |
| 저장 지도 로딩 성공 (map_server) | **CLI 검증됨** |
| Localization 노드(map_server+amcl) 활성화, `/amcl_pose` 갱신 | **CLI 검증됨** — 16절에서 AMCL로 전환 후 오차 0.11m까지 수렴 확인 (12절의 "수렴 불안정" 문제는 15-9/15-10 오도메트리 수정 이후 재검증하니 해결됨) |
| Nav2 lifecycle 노드 활성화 (controller/planner/bt_navigator 등) | **CLI 검증됨** |
| 목표 pose 수락 | **CLI 검증됨** (액션 accepted) |
| global path 생성 | **CLI 검증됨** (16절: AMCL 전환 후 두 목표 모두 경로 생성 성공) |
| `/cmd_vel` 출력, 로봇 실제 이동, 목표 도달 | **CLI 검증됨** (16-5절: 목표 2개 연속 `Goal succeeded`, ground truth로 실제 도달 확인) |
| RViz GUI에서 2D Pose Estimate/2D Goal Pose 조작 | **GUI 확인 필요** (이번 세션엔 CLI로 `/initialpose`, `action send_goal` 사용 — 사람이 직접 RViz에서 클릭하는 것은 미검증) |

## 15. 오도메트리 드리프트 원인 규명 및 수정 (이번 세션)

이전 세션까지는 "오도메트리가 가끔 튄다" 정도로만 문서화돼 있었는데, 이번 세션에서
`gz topic -e -t /world/aircore_world/dynamic_pose/info`로 **Gazebo 실제(ground truth)
좌표**를 직접 조회해서 `/odom`과 실측으로 대조했고, 원인 하나를 구체적으로 특정해서
고쳤다. 아래는 그 과정을 실제로 겪은 순서대로 적은 기록이다.

### 15-1. 증상: SLAM 지도에 같은 벽이 회전된 채로 방사형으로 겹쳐 찍힘

`wander.launch.py`로 몇 분 매핑한 뒤 지도를 저장해서 눈으로 보면, 중앙 교차로
윤곽이 부채꼴(방사형)로 여러 겹 찍혀서 벽이 뭉개져 보였다. (`gen_fake_map.py`가
만든 예전 가짜 맵이 아니라 실제 slam_toolbox가 만든 맵에서 발생.)

### 15-2. 원인 조사: ground truth 대 `/odom` 실측 대조

```bash
# Gazebo 실제 좌표(ground truth) 조회
gz topic -e -t /world/aircore_world/dynamic_pose/info -n 1 | grep -A9 'name: "aircore"'
# ROS2가 보는 오도메트리
ros2 topic echo /odom --once
```

측정 결과: 같은 시점에 ground truth는 원점에서 0.69m 떨어진 곳에 있는데
`/odom`은 원점에서 **6.9m** 떨어진 곳을 가리키고 있었다(7×7m 월드보다 큰 오차).

동시에 `wander.py`의 "탈출 기동" 로그를 자세히 보니, `front`/`left`/`right`
라이다 거리값이 **2초 간격으로 여러 번 완전히 동일**했다
(`front=0.77 left=1.11 right=0.90`이 그대로 반복). 로봇이 실제로 회전
중이라면 이 값이 계속 바뀌어야 하므로, 이건 로봇이 **물리적으로 완전히
멈춰 있는데 바퀴 관절만 명령대로 계속 도는 상태**라는 결정적 증거였다.

**결론**: `wander.py`의 탈출 기동이 후진(`linear.x=-0.08`)과 회전
(`angular.z=0.6`)을 **동시에** 명령하는데, 좁은 코너에서는 이 조합 자체가
기하학적으로 불가능해서(회전하려면 몸체가 양쪽 벽을 쓸고 지나가야 함) 로봇이
완전히 끼어버리고, 그 상태에서도 바퀴 관절은 명령대로 계속 돌아 오도메트리만
폭주한다.

### 15-3. 고친 것

| # | 파일 | 내용 |
|---|---|---|
| 1 | `urdf/aircore.gazebo` | `DiffDrive` 플러그인에 `max_angular_acceleration: 2.0` 추가. 각속도 제한이 없으면 명령이 바뀌는 순간 바퀴가 즉시 목표 회전속도로 점프해야 해서 순간적으로 마찰 한계를 넘어 미끄러질 수 있음. |
| 2 | `scripts/wander.py` | `safe_distance` 0.6 → 0.8m. 회전을 더 일찍 시작해서 몸체가 벽 모서리에 실제로 닿기 전에 피하게 함. |
| 3 | `scripts/wander.py` | 탈출 기동 방향을 무작위 대신 **라이다로 더 트인 쪽(left/right 비교)** 으로 고르도록 변경. 무작위면 같은 코너로 재진입할 확률이 그대로임. |
| 4 | `scripts/wander.py` | **탈출 기동을 2단계로 분리**: 처음 1.2초는 순수 직진 후진(각속도 0)으로 코너에서 몸부터 빼내고, 남은 1.3초 동안 그 자리에서 회전. 동시 명령이 만들던 기하학적 교착 상태를 없앰. |
| 5 | `config/mapper_params_online_async.yaml` | `minimum_travel_heading` 0.2 → 0.5rad. 제자리 회전(가장 오도메트리가 못 미더운 구간)마다 새 키프레임이 등록되던 것을 줄임. |
| 6 | `config/mapper_params_online_async.yaml` | `correlation_search_space_dimension` 0.5 → 1.0m. 오도메트리 오차가 커도 스캔 매칭이 재정렬을 시도할 수 있는 탐색 범위를 넓힘. |

적용 후 실측: 같은 조건에서 "탈출 기동" 발생 빈도가 약 10~20초당 1회 →
30~60초당 1회로 줄었고, 탈출 기동 직후 라이다 값이 스캔마다 계속 변하는
정상적인 패턴으로 바뀐 것을 확인함(더 이상 완전히 끼이지 않음).

### 15-4. 시도했지만 효과가 없었던 것: 물리 스텝을 1000Hz로 복원

"물리 스텝을 250Hz로 낮춘 게 접촉 해석 정밀도를 떨어뜨려 미끄러짐을 유발하는 게
아닐까" 가설로 `worlds/aircore_world.sdf`의 물리 스텝을 250Hz(4ms) → 1000Hz(1ms)로
복원해서 같은 방식(ground truth 대 odom)으로 재측정했다.

**결과는 예상과 반대였다**: 1000Hz에서는 로봇이 거의 움직이지 못했다 — 5초간
0.15m/s 전진을 명령해도 ground truth 이동량이 0.4cm에 불과했음(같은 명령을
250Hz에서 실행하면 0.58m 정상 이동). 이 로봇(가벼운 강체 + 저마찰 `mu=0.01`
캐스터를 고정 조인트로 근사한 구조)에는 오히려 250Hz가 더 안정적인 것으로
보여 250Hz로 원복했다. **정밀도를 올리는 게 항상 안정성을 올리는 건 아니라는
사례** — 이 조합에서는 접촉 솔버 반복 횟수 대비 스텝이 작아지면서 캐스터
고정 조인트 근사가 오히려 로봇을 "붙잡는" 쪽으로 작용한 것으로 추정됨(정확한
내부 메커니즘은 미확인).

### 15-5. 남은 한계: 여전히 비결정적인 잔여 드리프트

250Hz로 원복한 뒤에도, 장애물 없는 열린 공간에서 순수 직진만 시켜도 ground
truth 대비 `/odom` 이동 거리가 매번 다른 비율로 부풀려졌다:

| 테스트 | 명령 | ground truth 이동 | `/odom` 이동 | 비율(odom/gt) |
|---|---|---|---|---|
| 1회차 | 0.15m/s × 5초 | 0.582m | 0.838m | 1.44x |
| 2회차 | 0.15m/s × 8초 | 0.313m | 1.296m | 4.14x |

두 비율이 크게 다르다는 것(1.44배 vs 4.14배)은 `wheel_radius` 같은 상수
하나를 재보정해서 고칠 수 있는 **일정한 계산 오차가 아니라**, 매 구간마다
달라지는 **비결정적 바퀴-지면 접촉/미끄러짐**이라는 뜻이다. 캐스터가 진짜
자유회전 조인트가 아니라 저마찰 고정 조인트로 근사돼 있는 점(README.md,
SLAM_GUIDE.md에 이미 문서화됨)이 유력한 원인으로 보이지만, 이번 세션에서
근본적으로 없애지는 못했다 — **미해결로 문서화**.

**실용적 영향**: 코너에 끼여 완전히 멈춘 채 바퀴만 헛도는 극단적인 경우(15-2)는
고쳤지만, 평범하게 주행하는 동안에도 크든 작든 오도메트리 오차가 계속
누적되므로, 장시간 세션에서는 여전히 지도가 흐트러질 수 있다. 짧고 신중한
구간별 매핑(또는 필요하면 사람이 직접 조종)이 가장 안정적이다.

### 15-6. 추가로 찾은 원인: 바퀴 충돌체가 STL 원본 메시였음

`joint_states`를 직접 찍어보니 바퀴 관절이 **1000라디안 이상(약 162바퀴)**
회전했는데 그동안 ground truth 위치는 1m도 안 움직인 순간이 있었다. 이건
"가끔 미끄러지는" 수준이 아니라 **바퀴가 거의 완전히 헛돌고 있다**는 뜻이라
바퀴의 충돌(collision) 형상을 확인해봤다.

`urdf/aircore.xacro`의 `left_wheel`/`right_wheel` `<collision>`이 시각화용
`<visual>`과 똑같이 **STL 원본 메시**를 그대로 쓰고 있었다. ODE 같은
물리엔진에서 회전하는 바퀴의 충돌 형상으로 상세 메시를 쓰면 매 스텝 접촉점이
삼각형 단위로 들쭉날쭉 바뀌어서 마찰이 불안정해지는 게 잘 알려진 문제다(실제
로봇 시뮬레이션에서는 바퀴 충돌체로 원기둥/구 같은 단순 형상을 쓰는 게 정석).

**고친 것**: `left_wheel`/`right_wheel`의 `<collision>` 지오메트리를 STL
메시 대신 `<cylinder radius="0.0325" length="0.0918"/>`로 교체함(반지름은
기존 `DiffDrive` 플러그인 값과 동일, 길이는 STL 바운딩박스 Y축 실측값).
URDF cylinder는 로컬 Z축이 회전축이라 관절의 실제 회전축(Y축)에 맞춰
`rpy="1.5707963 0 0"`(X축 기준 90도)를 추가함. 캐스터(`support_wheel`)는
애초에 저마찰 근사라 그대로 둠.

### 15-7. 주의: 좀비 프로세스가 실험 결과를 오염시킨 사례

이 조사 중간에 `pkill -9 -f "gz sim"`로 Gazebo를 껐다고 생각했는데, 이 패턴이
`gz sim` 프로세스(서버/GUI)만 잡고 같은 `gazebo.launch.py`가 띄운
`robot_state_publisher`/`ros_gz_bridge`는 못 잡아서(다른 프로세스명이라
`-f "gz sim"`에 안 걸림) 좀비로 남았다. 그 상태에서 새로 Gazebo를 다시
띄우니 `ros2 node list`에 `/robot_state_publisher`, `/ros_gz_bridge`가
**2개씩** 잡혔고(4절의 §11-4와 동일한 증상), 그 세션에서 측정한 오도메트리
드리프트/지도 품질은 신뢰할 수 없다. **재현 시 반드시
`ros2 node list`로 중복 노드가 없는지 확인한 뒤 실험할 것.** 완전히 죽이려면
`pkill -9 -f "gz sim|robot_state_publisher|ros_gz_bridge|slam_toolbox|wander"`처럼
관련 프로세스명을 전부 나열하거나, `ps aux`로 PID를 직접 확인해서 지워야 한다.

### 15-8. 최종 결과 (원기둥 충돌체 적용 후, 좀비 프로세스 없이 재검증)

같은 방식(ground truth 대 `/odom`)으로 약 2분간 `wander` 주행 후 재측정:

| | 원점 대비 거리 |
|---|---|
| ground truth | 2.86m |
| `/odom` | 2.69m |

**방향(헤딩)은 여전히 어긋나 있지만(경로 자체가 다른 쪽으로 휘어 있음),
크기(magnitude)는 6% 오차로 크게 좁혀졌다** — 수정 전 관측된 7배~24배
오차(15-2, 15-5)에 비하면 수십 배 개선. 다만 지도를 눈으로 보면 여전히
벽 윤곽이 완전히 한 겹으로 깔끔하게 안 겹치고(방사형 잔상이 옅게 남음),
방향 드리프트가 남아 있다는 뜻이다. **완전히 해결되지는 않았고, 실용적으로
훨씬 나아진 수준으로 문서화한다.**

### 15-9. 진짜 근본 원인: 볼캐스터 높이가 바퀴보다 2cm 높게 떠 있었음 (사용자 발견)

15-6에서 캐스터 충돌체를 구(sphere)로 바꿨지만, 그때 STL 메시용으로 쓰던
`<collision><origin>` z-offset(-0.007)을 그대로 재사용했다. 이게 결정적인
실수였다: 그 offset은 "비대칭 STL 메시의 내부 좌표 원점"을 링크 원점에
맞추기 위한 값이라, 원래 자체가 대칭인 구(sphere)에는 안 맞았다.

**증상**: 사용자가 Gazebo 화면에서 로봇이 "위아래로 막 움직인다"고 직접
보고함. 실측(`gz topic -e /world/aircore_world/dynamic_pose/info`)해보니
정지 시엔 완벽히 안정적인데(z=0.0325 고정), 주행 중(회전 없이 직진만 해도)
z가 0.012~0.034 사이를 오가고 pitch가 최대 20도까지 흔들렸다.

**원인 계산**: 원기둥(바퀴)과 구(캐스터)의 충돌체 중심 위치를 관절 원점 +
collision origin으로 직접 계산해보니, 바퀴 바닥은 base_link 기준 -0.0325m,
캐스터 바닥은 -0.0125m — **캐스터가 바퀴보다 2cm 위에 떠 있어서 지면에
안 닿는 상태**였다. 그 결과 로봇이 사실상 바퀴 2개(가로 일직선)로만
지탱되는 구조가 되어, 앞뒤(pitch) 방향으로는 지지점이 없는 시소 상태가
됐다. 사용자가 정확히 "바퀴랑 높이 맞아야 한다"고 짚어낸 게 이 문제였다.

**고친 것**: `left_wheel`/`right_wheel`/`left_support_wheel`/
`right_support_wheel` 네 개 모두 `<collision><origin>`을 `xyz="0 0 0"`으로
바꿈 — 관절 원점(바퀴 z=0.027, 캐스터 z=0.007)이 이미 정확한 위치라서,
대칭 도형을 그 지점에 그대로 두면 별도 offset 없이 바닥이 정확히 두
경우 모두 -0.0055m로 일치함(문서화된 설계값과 정확히 같음).

**검증 결과**:
- 정지 시: z=0.005500으로 고정, pitch/roll 사실상 0 — 완벽히 안정.
- 직진 2초 + 즉시 정지: ground truth 0.309m 이동, `/odom` 0.308m 이동
  (오차 0.3%) — 수정 전(같은 조건에서 40~50%대 오차)과 비교해 극적으로
  개선.
- 직진+회전(0.4rad/s) 3초: ground truth 0.441m, `/odom` 0.425m (거리
  오차 4%), 다만 헤딩(진행 방향)은 약 12도 차이 — 거리 정확도는 크게
  좋아졌지만 방향(헤딩) 오차는 일부 남아 있음.
- `wander` 약 2분 주행 후: ground truth 원점 거리 3.68m, `/odom` 4.11m
  (오차 12%, 이전 세션 대비 크게 개선). 다만 저장된 지도를 보면 여전히
  벽 윤곽이 방사형으로 옅게 겹쳐 보임 — **거리(magnitude) 오차는 거의
  해결됐지만, 장시간 세션에서 누적되는 헤딩(방향) 드리프트는 남아있어서
  지도 품질에 계속 영향을 준다.**

**결론**: 이 15-9의 수정이 실제로는 15-2/15-5/15-8에서 씨름하던 "정체불명의
비결정적 드리프트"의 상당 부분을 설명한다 — 로봇이 주행 중 계속 앞뒤로
들썩였으니 바퀴 접지력이 불안정해서 미끄러졌던 것이다. 남은 헤딩 드리프트는
이 pitch 문제와는 별개의(더 작은) 원인일 가능성이 높고, 향후 조사 대상으로
남긴다.

### 15-10. 진짜진짜 근본 원인: 바퀴 충돌 원기둥의 폭이 실제 타이어보다 4.6배 넓었음

15-9를 고친 뒤에도 `wander`로 몇 분 이상 돌리면 지도에 여전히 방사형
잔상이 남았다. "회전(헤딩)만 따로 재보자"는 생각으로 로봇을 정지 상태에서
`angular.z=1.0`으로만 10초 돌리는 순수 회전 테스트를 해봤다 (직진 성분
없이 회전만 분리해서 오차 원인을 좁히기 위함):

- ground truth 회전량: 약 152.7도
- `/odom` 회전량: 약 21.9도 (15-9 수정 직후 최초 테스트 기준)

**실제 회전량이 오도메트리가 계산한 값보다 4.6배 컸다.** 코너를 돌 때마다
헤딩 추정치가 실제보다 훨씬 적게 갱신되니, 몇 번만 돌아도 SLAM이 생각하는
방향과 실제 방향이 크게 어긋나 지도가 방사형으로 흩어지는 것이다.

**원인 조사**: 15-6에서 바퀴 충돌체를 원기둥으로 바꿀 때, 길이(폭)를
STL의 Y축 전체 범위(8.2~100mm = 91.8mm)로 그대로 썼다. 그런데 STL을
회전축(Y) 기준 반지름 프로파일로 다시 분석해보니:

```
y=8~60mm   : 반지름 12.2mm로 일정  -> 사실은 얇은 "축/허브"
y=62~74mm  : 반지름 3.5~12.2mm     -> 허브에서 림으로 이어지는 스포크
y=74~100mm : 반지름 32.5mm         -> 진짜 타이어(림)는 여기부터
```

즉 반지름 32.5mm(타이어)는 STL의 바깥쪽 일부(y≈74~100mm, 약 20~26mm
폭)에만 있고, 안쪽 대부분(y=8~74mm)은 그보다 훨씬 가는 12.2mm 반지름의
축이었다. 그런데 91.8mm 원기둥을 32.5mm 반지름으로 통째로 만들면서,
로봇 중심선 근처(y=8mm)까지 뻗어나가는 비정상적으로 뚱뚱한 "바퀴"가
됐다. 이 오버사이즈 원기둥이 실제 접지 폭(그리고 그로 인한 유효
축간거리)을 왜곡시켜, `DiffDrive` 플러그인이 계산하는 각속도가 실제보다
훨씬 작게 나오는 근본 원인이었다.

**고친 것**: `left_wheel`/`right_wheel`의 원기둥 `length`를 `0.0918`
(91.8mm) → `0.02`(20mm)로 축소. STL 해상도가 낮아 타이어 폭 경계를 mm
단위로 정확히 특정하긴 어렵지만, 실측된 "반지름이 32.5mm로 완전히
고정되는 구간"(74~100mm, 최대 26mm) 안에서 보수적으로 20mm를 채택.
radius(32.5mm)와 origin(0,0,0, 관절 원점에 중심)은 그대로 유지.

**검증 결과**:
- 순수 회전 재테스트(수정 후): ground truth 152.7도, `/odom` 151.6도
  (오차 1도 미만, 이전 4.6배 오차에서 극적으로 개선).
- `wander`로 5분 이상 매핑: 지도가 방사형으로 안 흩어지고 **하나의
  일관된 십자 구조로 안정적으로 유지됨** — 여러 차례 재확인해도 지도
  크기/모양이 같은 상태로 유지되고 사본이 생기지 않음. 4개 방향 모두에서
  장애물(기둥 2개, 박스, 벽 홈) 디테일이 잡힘.
- ground truth 대 `/odom` 거리 오차는 세션 길이에 따라 여전히 0~30%
  정도 편차가 있지만(물리 엔진 접촉 노이즈 수준으로 추정), **지도가
  깨지거나 사본이 생기는 수준의 오차는 더 이상 발생하지 않음.**

**최종 결론**: 이번 세션에서 발견된 오도메트리 드리프트는 사실 **두 개의
독립적인 URDF 충돌 형상 버그**가 겹친 결과였다 —
1. (15-9) 캐스터가 바퀴보다 2cm 떠 있어서 주행 중 pitch로 계속 들썩임
2. (15-10) 바퀴 원기둥 폭이 실제 타이어보다 4.6배 넓어서 회전량이
   왜곡됨

두 개를 모두 고친 뒤에야 SLAM 지도가 안정적으로 수렴했다. `wander.py`의
반응형 탐색 알고리즘 자체는 좁은 복도 입구를 자주 못 뚫고 같은 구역만
맴도는 한계가 남아있지만, 이는 SLAM/오도메트리 버그가 아니라 별도의
탐색 전략 문제로 분류한다(12절 "현재 남아 있는 한계" 갱신 필요).

### 15-11. 새 지도로 Nav2 재검증 — 부분적 성공 (planner는 통과, controller는 실패)

15-9/15-10 수정과 새로 매핑한 지도(`maps/aircore_map.yaml/.pgm`, 십자
복도 + 4방향 일부 장애물)로 `nav2.launch.py`를 다시 띄우고 목표를
보내봤다.

**개선된 부분**: 로봇의 현재 추정 위치에서 가까운 목표(`(0.7, 1.1)`,
현재 위치 `(0.46, 1.03)`에서 약 0.3m)로 보내자 — 이전 세션(15절 이전)엔
`GridBased plugin failed to plan`으로 **전역 경로 생성 자체가 실패**
했었는데, 이번엔 전역 경로가 정상 생성되어 `controller_server`로
전달됐다("Passing new path to controller." 로그 반복 확인). 다만 너무
멀거나 지도 밖(회색/미탐색 영역)을 목표로 잡으면 여전히 "Failed to
create plan" 실패가 남(예: 원점 `(0,0)`으로 보냈을 때).

**여전히 실패하는 부분**: 컨트롤러(MPPI)가 "Failed to make progress"로
반복 중단됐다. `/cmd_vel`이 **단 한 번도 발행되지 않았고**(`ros2 topic
hz /cmd_vel` 확인 결과 발행 이력 없음), ground truth 위치도 시도 전후로
전혀 안 바뀜(1.122, -0.155 그대로) — 로봇이 아예 움직이려 시도조차
안 했다는 뜻이다.

**원인**: `tf2_echo map base_link`로 확인한 Nav2의 "현재 위치" 추정치는
`(0.456, 1.031)`인데, 같은 시점 `gz topic -e`로 확인한 ground truth는
`(1.122, -0.155)` — **약 1.4m 차이**. 15-9/15-10으로 오도메트리 오차를
수십 배 줄이긴 했지만, 남은 잔여 드리프트(0~30% 수준, 15-10 참고)가
누적되어 이 정도 위치 오차를 만들고 있다. Nav2의 MPPI 컨트롤러는 이
(잘못된) 현재 위치 기준으로 경로 추종을 시도하다가 진전이 없다고
판단해 즉시 포기하는 것으로 보인다.

**결론**: 사용자가 지적한 대로("이러면 NAV2를 하는게 의미가 없잖아")
근본적으로 맞는 지적이었다 — 오늘 고친 두 버그로 SLAM 지도 품질은
확실히 좋아졌지만(더 이상 깨지지 않음), **Nav2의 목표 주행 실행은
여전히 잔여 오도메트리 드리프트에 막혀 있다.** 전역 계획(planner)까지는
확실히 개선됐고 실제로 통과하는 걸 확인했지만, 지역 제어(controller)
단계에서 막힌다. 다음 세션에서 우선순위 높은 작업:
1. 잔여 헤딩 드리프트의 남은 원인을 마저 찾기(15-10에서 못다 한 정밀
   보정, 혹은 캐스터 마찰/충돌 안정성 추가 튜닝).
2. 또는 AMCL + 저장된 정적 지도 조합으로 전환해서, slam_toolbox의
   실시간 `map→odom` 보정보다 더 안정적인 로컬라이제이션을 시도.
3. 그때까지는 "목표 지점까지 자율주행 성공"은 **미해결**로 유지.

## 16. Nav2 목표 주행 최종 성공 — slam_toolbox 대신 AMCL+저장 지도로 전환

15-11 직후 같은 세션에서 바로 검증했다.

### 16-1. 지도 보강: 8개 장애물 전부 주변을 돌아 뒤편까지 스캔

`wander.py`(반응형 랜덤워크) 대신, `worlds/aircore_world.sdf`에 정의된
장애물 좌표(NE 박스 2개, NW 기둥 2개, SE 박스 2개, SW 기둥+L벽)를 직접
읽어서 각 장애물 주변을 원형으로 도는 웨이포인트(`/odom` 피드백 기반
비례제어)를 짜서 실행했다. "장애물 뒤가 안 보인다"는 지적에 대한 조치 —
2D 라이다는 직선 시야만 보므로, 뒤쪽을 채우려면 로봇이 실제로 돌아가서
그 각도에서 스캔해야 한다(라이다 자체의 근본적 한계이지 버그가 아님).
8개 장애물을 전부 순회한 뒤 지도를 저장해서 `maps/aircore_map.yaml/.pgm`을
덮어썼다(245×261 픽셀, 4방향 장애물 디테일 포함). 다만 원을 그리며 도는
동작 자체가 회전이 많아 잔여 헤딩 드리프트가 다시 늘어서, 이 지도는
15-10 직후의 가장 깨끗했던 지도보다는 약간 번져 있다.

### 16-2. 시스템 과부하로 인한 오작동 — 버그 아님을 확인

이 순회 도중 사용자가 "차가 순간이동한다", "Gazebo에서 로봇이 안
움직인다"고 보고했다. ground truth를 3초 간격으로 두 번 조회해서
실제로 물리적으로 연속 이동 중임을 확인했다(예: (0.212,0.185) →
(0.449,0.090), 3초간 약 0.26m 이동 — 정상). 원인은 `uptime`으로 확인한
load average 14.84(정상은 CPU 코어 수 이하) — RViz2가 오래 켜져 있으면서
CPU 시간을 707분까지 누적하며(메모리/렌더링 부하 추정) 시스템 전체를
과부하 상태로 만들었고, 그로 인해 Gazebo GUI 렌더링이 프레임을 건너뛰어
"순간이동"처럼 보인 것이었다. RViz를 강제 종료 후 재시작하고, 이후에는
Gazebo도 헤드리스(`headless:=true`)로 돌려서 완화함.

### 16-3. 진짜 병목 재확인: slam_toolbox의 map→odom이 이번엔 3.5m까지 벌어짐

장애물 순회처럼 회전이 많은 장시간 세션 직후 `tf2_echo map base_link`로
확인한 Nav2의 위치 추정(`(-0.636, 0.456)`)과 같은 시각 ground truth
(`(2.732, -0.405)`)를 비교하니 **약 3.5m** 차이가 났다 — 15-11에서 관찰한
1.4m보다 더 벌어짐(장애물 순회의 회전량이 많았던 만큼 헤딩 드리프트가
더 누적된 것으로 보임). 이 상태로는 Nav2가 절대 정상 동작할 수 없다고
판단.

### 16-4. 해결: slam_toolbox 실시간 보정 → AMCL + 저장 지도로 교체

`slam.launch.py`를 끄고 `localization.launch.py`(`map_server` + `amcl`,
16-1에서 저장한 지도 사용)로 교체했다. slam_toolbox는 이전 추정치 위에
매 스캔 증분 보정을 쌓아가는 방식이라 한 번 크게 틀어지면 스스로 복구하기
어려운 반면, AMCL은 파티클 필터로 **현재 라이다 스캔과 저장된 지도 전체를
매번 다시 대조**하므로 이전 오차를 짊어지지 않고 훨씬 안정적으로 수렴한다.

`/initialpose`로 대략적인 시작 위치를 한 번 알려준 뒤 검증:
- 정지 상태 시드 직후: AMCL과 ground truth 거의 완전 일치
- 0.5m 직진 이동 후: ground truth (0.521, 0.000) vs AMCL (0.410, -0.024)
  — 오차 0.11m (15-11의 slam_toolbox 방식 1.4~3.5m 대비 극적으로 개선)

### 16-5. Nav2 목표 주행 최종 성공 (재현 확인)

AMCL 활성화 후 `nav2.launch.py`로 목표 세 개를 연달아 보냈다:

| 목표 | 결과 | 도달 위치(ground truth) | 오차 |
|---|---|---|---|
| (1.5, 0.0) | **Goal succeeded** | (1.68, -0.32) | ~0.36m |
| (0.0, -1.5) | **Goal succeeded** | (-0.46, -1.28) | ~0.55m |
| (-1.5, -1.5) | **Goal succeeded** | (-1.50, -1.49) | **~0.013m** |

세 시도 모두 `/cmd_vel`이 정상 발행됐고(약 20Hz), 로봇이 실제로
움직였고, `bt_navigator`가 `Goal succeeded`를 로그로 남겼다. 첫 시도에서
중간에 한 번 `Failed to make progress`가 났지만 bt_navigator가 자동으로
재시도(replanning)해서 결국 성공했다 — 15-11 때는 재시도해도 매번
실패했던 것과 대조적이다. 세 번째 목표(SW 방향, 코너를 도는 더 긴
경로)는 1.3cm 오차로 정밀하게 도달해서, AMCL 전환이 일회성 요행이
아니라 안정적으로 재현됨을 확인했다.

**최종 결론**: Nav2 목표 주행 실패의 진짜 원인은 로봇 URDF나 Nav2
설정이 아니라, **slam_toolbox의 실시간 보정이 장시간·회전이 많은
세션에서 큰 오차로 발산하면 스스로 복구하지 못한다**는 것이었다.
매핑은 slam_toolbox로 하되(생성 단계), 실제 자율주행 실행은 저장된
지도 + AMCL 조합으로 전환하는 것이 훨씬 안정적이다. 이 패턴(SLAM으로
지도 생성 → 지도 저장 → AMCL로 전환해서 주행)이 이 로봇/환경 조합에서는
사실상 필수적인 운영 절차로 확인됐다.

## 17. 처음부터 완전히 재현 — 커스텀 웨이포인트 스크립트의 함정과 최종 검증

사용자 요청으로 오도메트리 리셋부터 전체 파이프라인을 다시 실행하며
재현성을 검증했다.

### 17-1. 커스텀 웨이포인트 순회 스크립트가 벽에 로봇을 가둔 사고

16절에서 장애물 뒤편을 스캔하려고 만든 커스텀 웨이포인트 순회
스크립트(`/tmp/waypoint_driver.py`, 오도메트리 기반 비례제어)를 재사용했는데,
이번엔 로봇이 벽에 붙은 채로 여러 웨이포인트를 "도달"한 것으로 잘못
기록되며 계속 진행됐다. 원인: 이 스크립트는 `wander.py`와 달리 **막힘
감지/탈출 로직이 전혀 없다.** 벽에 부딪히면 바퀴가 헛돌면서(실제 이동
없음) 오도메트리만 슬쩍 부풀어 "도달 판정" 임계값(0.15m)을 넘기고, 다음
목표로 넘어가버린다. 실측: `/odom`은 (2.36, -1.69)라고 믿는데 ground
truth는 (-3.35, 0.20) 서쪽 벽 근처 — **6m 차이**. 지도가 "지저분하게"
보인다는 사용자 지적의 원인이 바로 이 오차였다(SLAM 알고리즘 자체는
정상; 입력으로 들어가는 위치 추정이 잘못됐을 뿐).

**교훈**: 자동 주행 스크립트를 새로 짤 때는 반드시 (a) 오도메트리 기반
"도달 판정"과는 별개로 실제 충돌/정체를 감지하는 안전장치, (b) 후진과
회전을 분리한 탈출 기동(15절의 wander.py 로직 참고)이 있어야 한다.
없으면 이런 사고가 재현된다. **이후 재매핑은 검증된 `wander.py`로만
진행**했다.

### 17-2. 프로세스 종료(pkill/kill) 명령이 조용히 실패하는 문제 재확인

이 세션 중간에 `pkill -9 -f <패턴>`과 `kill -9 <PID들>`이 반환값(exit
code 9)만 보고 "죽었겠지" 하고 넘어갔다가, 몇 번이고 이전 Gazebo/RViz/
Nav2 인스턴스가 **여전히 살아있는 채로 새 인스턴스와 공존**하는 사고가
반복됐다. 그 결과 `ground truth`/`/odom`/`/amcl_pose`가 서로 완전히
다른 값을 가리키는 극도로 혼란스러운 상태에 빠졌다(예: 세 소스가 각각
(3.35,-0.16), (-10.91,5.81), (-3.92,2.73)를 가리킴 — 물리적으로 불가능한
수준의 불일치). **교훈**: `pkill`/`kill` 실행 후에는 반드시 `ps aux`로
직접 재확인해야 하며, exit code만으로 성공 여부를 판단하면 안 된다.
이 문제는 이 세션 내내 반복된 근본 원인이었다(15-7절에서 처음 발견,
17절에서 재확인).

### 17-3. Nav2 `collision_monitor`가 안전상 `/cmd_vel`을 덮어써서 수동 복구를
막은 사례

로봇이 벽에서 0.15m 이내로 붙은 상태에서 Nav2가 켜져 있으면,
`collision_monitor`가 `/cmd_vel`에 안전 정지(0속도)를 계속 publish해서
**수동으로 후진 명령을 보내도 무시된다** (`ros2 topic info /cmd_vel -v`로
`collision_monitor`가 `/cmd_vel`의 두 번째 publisher임을 확인). 벽에 낀
로봇을 수동으로 빼내려면 먼저 `collision_monitor`(또는 Nav2 스택 전체)를
잠깐 내려야 한다.

### 17-4. 완전 재시작 후 최종 재현 성공

위 문제들을 정리한 뒤 Gazebo/SLAM/RViz/wander를 전부 처음부터
재시작하고(오도메트리·지도 완전 리셋), `wander.py`로만 5~9분 매핑,
지도 저장, `slam_toolbox` → AMCL 전환, `/initialpose`로 시드(오차
0.03m로 수렴), Nav2 목표 두 개를 연속으로 보냈다:

| 목표 | 결과 | 도달 위치(ground truth) | 오차 |
|---|---|---|---|
| (-0.5, 1.3) | **Goal succeeded** | (-0.500, 1.373) | 0.073m |
| (1.0, -1.0) | **Goal succeeded** | (1.026, -0.928) | 0.077m |

16절의 결과가 일회성이 아니라 **"SLAM(wander.py)으로 지도 생성 → 지도
저장 → AMCL로 전환해서 주행"** 절차를 그대로 재현하면 항상 재현된다는
것을 처음부터 다시 확인했다.
