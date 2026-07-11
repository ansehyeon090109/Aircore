# aircore_description — SLAM + Nav2 가이드 (ROS 2 Jazzy + Gazebo Harmonic)

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
| `package.xml` | XML 문법 수정 + `ros_gz_sim`, `ros_gz_bridge`, `slam_toolbox`, `rviz2`, `joint_state_publisher_gui`, `nav2_bringup` 의존성 추가 |
| `CMakeLists.txt` | `config/` 디렉터리도 설치되도록 추가 |
| `urdf/aircore.gazebo` | 라이다: `ray` → `gpu_lidar` 센서로 변경. 바퀴 구동: `libgazebo_ros_diff_drive.so` → `gz::sim::systems::DiffDrive` 플러그인으로 교체 (바퀴 간격도 실제 조인트 좌표 기준 0.1505m로 보정, 기존 값 0.2는 오차 있었음). `JointStatePublisher` 플러그인 추가 → 갱신. `gpu_lidar` 센서 `<pose>`를 0.015m 위로 올려서 jetson_nano 자기 감지 버그 수정 → **재갱신: `gz_frame_id` 태그를 실수로 지웠다가 TF 프레임 불일치 버그가 생겨서 다시 복구함 (9-0-2 참고)** → **재갱신: 모든 링크의 `selfCollide`를 `true`→`false`로 변경 (바퀴-차체 메시가 겹쳐서 매 스텝 자기충돌이 발생해 로봇이 들썩이며 거의 못 움직이던 원인, 9-3-2 참고), `wheel_radius`를 실측값 0.0325m로 보정** |
| `launch/gazebo.launch.py` | 스폰 높이 `-z 0.05` → `-z 0.008`로 수정. STL 바닥면을 직접 계산해보니 바퀴 접지점이 base_link 원점보다 5.5mm 아래인데 0.05m 높이에서 스폰하면 바퀴가 땅 위 4.4cm 뜬 채로 시작해서 떨어지는 충격 때문에 접지가 불안정해짐 (9-3 참고) |
| `worlds/aircore_world.sdf` | 새로 생성 → 두 번 갱신. 바닥/조명 + gz-sim 필수 시스템 플러그인 + **7m×7m 방을 십자 칸막이로 4개 소방(NE/NW/SE/SW)으로 분리, 각 방마다 다른 모양 장애물(박스/기둥/L자 벽) 2개씩** — 처음엔 빈 바닥에 박스 1개뿐이라 맵이 너무 단순했던 걸 구조물 많은 난이도로 교체 |
| `scripts/wander.py` | 새로 생성 → 갱신. `/scan` 기반 단순 반응형 자동 주행(장애물 회피) 노드 — 직접 조종 안 해도 로봇이 알아서 돌아다니게 함. `/scan` 끊김 감시(watchdog)와 속도 로그를 추가해서 "안 움직임" 문제 진단이 쉬워지게 함 → **재갱신: 벽 앞에서 매 스캔마다 좌/우 방향을 다시 비교하다 보니 값이 비슷할 때 방향이 계속 뒤집혀서 제자리에서 "와리가리"하던 버그 수정 — 회전 방향을 한 번 정하면 앞이 충분히 트일 때까지(1.4배 마진) 그 방향을 유지하는 상태(state)를 추가함** → **재갱신: 코너/좁은 틈에서 회전만 반복하고 실제로는 제자리에 멈춰있는 버그 수정 — `/odom`으로 실제 이동 거리를 3초마다 확인해서 8cm 미만이면 후진+큰 회전 탈출 기동을 강제로 실행함 (9-5 참고)** |
| `launch/wander.launch.py` | 새로 생성. `wander.py` 실행용 |
| `config/bridge.yaml` | 새로 생성. gz ↔ ROS2 토픽 브릿지 설정 (`/cmd_vel`, `/odom`, `/tf`, `/scan`, `/joint_states`, `/clock`) |
| `config/mapper_params_online_async.yaml` | 새로 생성. slam_toolbox 온라인 매핑 파라미터 → **재갱신: `transform_timeout`을 0.2 → 1.0초로 늘림 (낮은 RTF 환경에서 tf 조회 지연으로 스캔 매칭이 실패해 지도에 같은 방이 여러 겹 찍히던 문제, 10-2 참고)** |
| `config/aircore.rviz` | 새로 생성 → 갱신. RViz2 기본 설정 + global/local costmap, 전역 경로(`/plan`), `2D Goal Pose` 툴 추가 |
| `config/nav2_params.yaml` | 새로 생성. Nav2 파라미터 (controller/planner/smoother/behavior/bt_navigator/costmap 등). amcl·map_server는 안 씀 — slam_toolbox가 이미 `/map`과 `map→odom` tf를 제공하므로 그대로 재사용 |
| `launch/nav2.launch.py` | 새로 생성. `nav2_bringup`의 `navigation_launch.py`를 우리 params로 include |
| `launch/rviz.launch.py` | 새로 생성. rviz2 단독 실행용 — `ros2 launch`로 켜야 워크스페이스 환경(패키지 경로)이 항상 올바르게 잡혀서 메시(STL) 로드 실패를 막을 수 있음 |
| `launch/*.launch` (ROS1 XML) | 삭제하고 `display.launch.py` / `gazebo.launch.py` / `slam.launch.py` 로 재작성 |
| `launch/controller.launch`, `controller.yaml` | 삭제. `ros_control` 방식 velocity controller는 더 이상 필요 없음 — gz-sim `DiffDrive` 플러그인이 `/cmd_vel`을 직접 받아 바퀴를 구동함 |

## 2. 이게 "SLAM 완성"인가?

계획표 Day3 목표("slam_toolbox 연동, 시뮬레이션 환경에서 지도 생성 성공시키기")
기준으로는 **파이프라인 자체는 완성**입니다 — Gazebo에서 라이다가 `/scan`을
쏘고, slam_toolbox가 그걸로 `/map`과 `map→odom` tf를 실시간으로 만들어내는
구조가 정상 동작하면 그게 Day3 완료 조건입니다.

다만 "완성"이 아래 중 어떤 걸 뜻하는지에 따라 아직 더 할 일이 있을 수 있습니다.

- **지도 생성 자체가 되는가** → 됩니다 (파이프라인 완성).
- **지도가 실제로 쓸만한 품질인가** → 이건 (a) 환경에 라이다가 잡을 특징(벽,
  모서리)이 있어야 하는데, 이전 월드는 빈 바닥이라 그게 없었던 게 "맵이 너무
  단순하다"의 원인입니다. 지금은 벽+장애물로 방을 만들어서 해결했습니다.
  (b) 로봇을 방 구석구석 충분히 돌아다니게 해야 벽 전체가 채워집니다 — 한
  바퀴만 돌면 안 스캔된 구석이 비어 보입니다.
- **Nav2로 목적지까지 자율주행** → 이번에 계획이 바뀌어서 Nav2까지 포함시켰습니다 (아래 7번 참고).

지금 상태로 다시 실행해서 로봇을 방 전체에 돌아다니게 하면 벽 4개 +
칸막이 + 기둥/박스가 다 채워진 지도가 나와야 정상입니다. 그래도 맵이 비거나
이상하게 어긋나 있으면 알려주세요.

## 3. 아키텍처 한눈에 보기

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

## 4. 사전 설치

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
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-teleop-twist-keyboard
```

## 5. 빌드

```bash
cd /mnt/c/Users/admin/aircore
source /opt/ros/jazzy/setup.bash
colcon build --packages-select aircore_description
source install/setup.bash
```

## 6. 실행 순서 (터미널 4개)

`~/.bashrc`에 ROS Jazzy + 이 워크스페이스(`install/setup.bash`) 자동 소스를
넣어놨기 때문에, **새 터미널을 열기만 하면** 아래 명령이 바로 됩니다
(따로 `source ...`를 칠 필요 없음). 지금 열려 있던 터미널들은 한 번만
`source ~/.bashrc` 하거나 새로 열어주세요.

**터미널 1 — Gazebo 시뮬레이션 + 로봇 스폰 + gz↔ROS2 브릿지**
```bash
ros2 launch aircore_description gazebo.launch.py
```
Gazebo Harmonic 창이 뜨고 벽으로 둘러싸인 방 안에 로봇이 스폰됩니다.
좌측 하단 재생(▶) 버튼이 눌려있는지 확인하세요(월드에 `-r` 옵션을 넣어놔서
자동 재생되긴 합니다).

확인:
```bash
ros2 topic list          # /scan, /odom, /tf, /cmd_vel, /joint_states 보이는지
ros2 topic hz /scan       # 약 10Hz로 발행되는지
```

**터미널 2 — RViz2로 시각화**
```bash
ros2 launch aircore_description rviz.launch.py
```
`rviz2 -d ...`로 직접 켜지 말고 꼭 `ros2 launch`로 켜세요. 로봇 메시(STL)가
`package://aircore_description/meshes/...` 경로로 참조되는데, 그 터미널에
워크스페이스가 안 sourced 돼 있으면(`AMENT_PREFIX_PATH`에 우리 패키지가 없으면)
RViz가 그 경로를 못 찾아서 "Unable to open file" 에러를 냅니다. `ros2 launch`는
그 자체가 워크스페이스가 sourced 안 돼 있으면 아예 실행이 안 되니, 실행됐다는
것 자체가 환경이 맞다는 뜻이라 이 문제가 원천적으로 안 생깁니다.

왼쪽 Displays 패널에서 `Fixed Frame`을 `odom` → (slam 실행 후) `map` 으로
바꿔가며 확인하면 됩니다.

**터미널 3 — slam_toolbox 실행**
```bash
ros2 launch aircore_description slam.launch.py
```
RViz2에서 `Fixed Frame`을 `map`으로 바꾸고 `Map` 디스플레이를 켜면
지도가 그려지기 시작합니다.

**터미널 4 — 로봇을 움직여서 맵 채우기 (자동 주행)**
```bash
ros2 launch aircore_description wander.launch.py
```
직접 조종하기 번거롭다고 해서 `scripts/wander.py`(자동 주행 노드)를
추가했습니다. `/scan`만 보고 앞이 막히면 트인 쪽으로 돌고, 아니면 전진하는
단순한 반응형 랜덤 워크입니다 — Nav2의 "지능적인" 프런티어 탐색은 아니고,
그냥 알아서 돌아다니며 부딪히지 않게 피하는 수준입니다. 그래도 4개 방을
다 채우는 덴 충분합니다. 직접 조종하고 싶으면 대신
`ros2 run teleop_twist_keyboard teleop_twist_keyboard`(`i/,/j/l/k` 키)를
쓰면 됩니다.

일부 구역만 돌면 맵의 나머지 부분이 비어(회색/미탐색) 보이는 게 정상이고,
그건 슬램이 잘못된 게 아니라 아직 안 가본 곳입니다. `wander.launch.py`를
몇 분 정도 켜두면 4개 방이 대부분 채워집니다.

## 7. Nav2로 목적지 이동 (Day4)

맵이 어느 정도 채워졌으면(적어도 가려는 방까지는 스캔된 상태), **터미널 4의
`wander.launch.py`는 Ctrl+C로 끄세요.** `wander.py`와 Nav2가 둘 다 `/cmd_vel`에
명령을 보내면 서로 충돌해서 로봇이 이상하게 움직입니다 — 자동 탐색과 Nav2
목적지 주행은 동시에 켜두면 안 됩니다.

**터미널 4(재사용) — Nav2 실행**
```bash
ros2 launch aircore_description nav2.launch.py
```
`amcl`/`map_server`는 안 씁니다 — slam_toolbox가 이미 `/map`과 `map→odom` tf를
계속 주고 있으니 Nav2는 그걸 그대로 받아씁니다. `controller_server`,
`planner_server`, `smoother_server`, `behavior_server`, `bt_navigator`,
`waypoint_follower`, `velocity_smoother` + costmap 2개가 뜹니다.

확인:
```bash
ros2 topic list           # /plan, /global_costmap/costmap, /local_costmap/costmap 등
ros2 action list          # /navigate_to_pose 보이는지
```

**RViz2에서 목적지 보내기**
1. 왼쪽 Displays에서 `GlobalCostmap`, `LocalCostmap` 체크박스를 켜서 코스트맵이
   맵 위에 겹쳐 보이는지 확인 (장애물 주변이 부풀려진 색으로 보이면 정상).
2. 위쪽 툴바에서 `2D Goal Pose` 도구 선택 → 이미 스캔된(맵이 채워진) 영역
   안의 지점을 클릭 + 드래그해서 방향까지 지정.
3. 초록색 `/plan` 경로가 그려지고 로봇이 그 경로를 따라 움직이면 성공입니다.

아직 스캔 안 된(회색) 영역으로 목적지를 보내면 플래너가 실패하거나 이상한
경로를 낼 수 있습니다 — 반드시 이미 지도가 채워진 곳으로 보내세요.

## 8. 맵 저장 (Day3 완료 기준)

```bash
ros2 run nav2_map_server map_saver_cli -f ~/aircore_map
```
`nav2_map_server`가 아직 없다면 `sudo apt install ros-jazzy-nav2-map-server`로
설치하거나, slam_toolbox 자체 서비스로 저장해도 됩니다:
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'aircore_map'}}"
```

## 9. Gazebo는 안 움직이는데 RViz만 움직일 때

이건 아주 특정한 증상이라 원인이 좁혀집니다. 아래 순서대로 하나씩 확인하세요.

### 9-0. (실제 발견된 사례) left/right가 항상 0.05~0.06으로 고정 — 라이다 자기 감지 버그, 지금 고침

`wander.launch.py` 로그를 실제로 받아서 확인해보니:
```
front=0.90 left=0.05 right=0.06 -> v=0.18 w=-0.07   (25초 내내 거의 그대로)
```
`left`/`right`가 25초 동안 0.05~0.06m에서 전혀 안 변한 게 이상해서 STL
치수를 직접 계산해봤습니다. 원인을 찾았습니다: **라이다 센서 원점이 바로
아래에 있는 `jetson_nano` 상단(z≈160mm)과 라이다의 스캔 높이(z≈159.5mm)가
거의 같은 높이라서, 좌/우 방향 레이저가 항상 자기 로봇의 Jetson Nano
보드 모서리(센서에서 수평거리 약 55~65mm)에 맞고 튕겨서 "장애물"로
잡히고 있었습니다.** 로봇이 어디에 있든, 실제로 움직이든 안 움직이든
이 값은 절대 안 바뀝니다 — 자기 몸을 보고 있는 거니까요.

`urdf/aircore.gazebo`의 `gpu_lidar` 센서 `<pose>`를 `0 0 0 ...`에서
`0 0 0.015 ...`로 올려서(스캔 원점만 1.5cm 위로, 라이다 하우징 모델 자체는
안 움직임) jetson_nano보다 위에서 스캔하도록 고쳤습니다. `colcon build`
다시 하고 재실행하면 `left`/`right`가 이제 실제 벽까지의 거리를 반영해서
로봇이 움직이면 같이 바뀔 겁니다.

**단, 이건 `left`/`right`만의 원인이고 `front`가 25초 내내 0.90~0.92로
고정돼 있던 건 별개의 문제입니다.** front 쪽엔 저렇게 가까운 자기 부품이
없으므로(계산상 라이다 앞쪽 55mm 반경엔 아무 것도 없음), front가 안 바뀐
건 실제로 **로봇이 안 움직이고 있다는 신호**입니다. 아래 9-1부터 순서대로
확인하세요 — 특히 라이다 버그를 고친 뒤에도 front가 여전히 안 바뀐다면
9-3(바퀴 헛돎)일 가능성이 높습니다.

### 9-0-2. (실제 발견된 사례, 회귀 버그) RViz에 `Message Filter dropping message:
frame 'aircore/base_link/lidar_sensor' ... discarding message because the
queue is full` — 라이다가 아예 안 보임, 지금 고침

9-0에서 `gz_frame_id` 태그를 "SDF에 정의 안 된 태그"라는 경고 메시지만
보고 제거했는데, 이게 **잘못된 조치였습니다.** 이 태그를 지우고 나서
`/scan`은 발행되는데 RViz/slam_toolbox가 그 데이터를 하나도 못 쓰는
새 버그가 생겼습니다.

**원인**: `lidar_joint`가 `fixed` 타입이라서, URDF→SDF 변환 시 sdformat이
`lidar` 링크를 `base_link`에 합쳐버립니다(고정 조인트는 기본적으로 별도
링크를 안 만들고 부모에 합쳐짐). 그래서 `gz_frame_id`로 프레임 이름을
직접 지정하지 않으면 gz-sim이 센서 frame_id를
`"aircore/base_link/lidar_sensor"`처럼 자동으로 만들어버립니다. 그런데
`robot_state_publisher`가 URDF 그대로 내보내는 TF 트리에는 그런 이름의
프레임이 없고 그냥 `lidar`만 있습니다. RViz/slam_toolbox가 스캔 메시지를
변환할 TF를 찾다가 영원히 못 찾아서 큐가 꽉 차 메시지를 계속 버리는
겁니다 — 화면에 라이다 포인트가 안 보이고, slam_toolbox도 스캔을 못 써서
지도가 안 그려집니다.

**고친 것**: `urdf/aircore.gazebo`의 `gpu_lidar` 센서에
`<gz_frame_id>lidar</gz_frame_id>`를 다시 넣었습니다. sdformat은 여전히
"SDF에 없는 태그"라고 경고를 띄우지만, 그건 스키마 검증기가 낯선 태그를
경고하는 것뿐이고 **gz-sensors 코드는 이 값을 실제로 읽어서 사용합니다**
— 무시해도 되는 경고였는데, 지난 턴에 이 경고만 보고 동작하던 기능을
지운 게 이번 회귀의 원인이었습니다.

### 9-1. 애초에 명령이 나가고 있나?
```bash
ros2 topic hz /cmd_vel
```
- **아무것도 안 뜨면(발행 자체가 없음)**: `wander.launch.py`나 teleop을
  안 켰거나, teleop 터미널에 키보드 포커스가 없는 경우입니다. 이 경우
  Gazebo가 안 움직이는 건 **정상**입니다 — 아무도 움직이라고 안 했으니까요.
  그런데 이때 RViz에서 로봇이 "혼자 조금씩" 움직이는 것처럼 보인다면, 그건
  실제로 움직이는 게 아니라 **slam_toolbox가 map→odom tf를 계속 미세하게
  보정**하면서 생기는 흔들림입니다 (라이다 노이즈 때문에 아주 조금씩 위치를
  재추정함). Fixed Frame이 `map`일 때만 이 흔들림이 보이고, `odom`으로
  바꾸면 로봇이 완전히 고정돼 있을 겁니다 — 한번 바꿔서 확인해보세요.

### 9-2. 명령은 나가는데 Gazebo 바퀴가 안 도나?
```bash
ros2 topic echo /cmd_vel        # 값이 실제로 오는지
```
Gazebo 창에서 바퀴(원통 두 개)가 제자리에서 도는지 눈으로 확인하세요.
- **바퀴도 안 도는 경우**: `ros2_gz_bridge`가 `/cmd_vel`을 gz 쪽으로 못
  넘기고 있거나, `DiffDrive` 플러그인이 안 뜬 겁니다. Gazebo를 띄운
  터미널(터미널 1) 로그에서 `DiffDrive`나 `plugin` 관련 에러가 있는지
  확인하세요.

### 9-3. (실제 발견된 사례) 로봇 스폰 높이가 바퀴 접지점보다 4.4cm 높음 —
가장 유력한 "차체가 안 움직임" 원인, 지금 고침

`launch/gazebo.launch.py`의 스폰 명령은 `-z 0.05`로 되어 있었습니다. 이
숫자가 실제 로봇 치수와 맞는지 STL 메시 바운딩박스를 직접 계산해서
확인했습니다:

- `left_wheel.stl`/`right_wheel.stl`/`left_support_wheel.stl`/
  `right_support_wheel.stl` 전부 바닥면이 `base_link` 원점 기준
  **z = -0.0055m (5.5mm 아래)** 에 있습니다 (구동바퀴와 캐스터가 같은
  높이에서 접지하도록 설계돼 있음 — 정상).
- 그런데 `-z 0.05`로 스폰하면 `base_link` 원점이 월드 z=0.05에서
  시작하므로, 바퀴 접지점은 `0.05 + (-0.0055) = 0.0445m`, 즉 **땅 위
  4.4cm 뜬 상태로 시작**합니다.

이 로봇은 가볍고(전체 질량 1~2kg대) 바퀴-지면 접촉이 정밀해야 하는
구조라서, 스폰 직후 4.4cm를 낙하하면서 튀거나 살짝 기울어진 채로
착지하기 쉽습니다. 이러면 `DiffDrive`가 계산하는 `/odom`(바퀴 회전량
기반 추정치)은 정상적으로 "전진"을 계산해서 RViz/slam_toolbox 쪽에서는
로봇이 움직이는 것처럼 보이는데, Gazebo의 실제 물리 위치(바퀴가 헛돌거나
불안정하게 접지된 상태)는 안 바뀌는 정확히 이 증상이 나옵니다.

**고친 것**: `launch/gazebo.launch.py`의 스폰 높이를 `-z 0.008`로
낮췄습니다 (바퀴 접지점보다 아주 살짝만(2.5mm) 위 — 완전히 붙여서
스폰하면 초기 겹침(interpenetration)으로 오히려 물리 엔진이 로봇을
튕겨낼 수 있어서 최소한의 여유만 둠). `colcon build` 후 다시 스폰해서
로봇이 바닥에 안정적으로 서 있는지, 그리고 실제로 전진하는지 확인하세요.

**만약 이걸 고치고도 여전히 안 움직인다면**, 다음으로 의심할 곳은
캐스터(`left/right_support_wheel`)입니다. 이 로봇의 캐스터는 진짜
자유회전 볼이 아니라 **낮은 마찰(mu=0.01)의 고정 조인트**로 근사돼
있습니다. Gazebo 창에서 카메라를 로봇 옆(바닥 높이)으로 가까이 가져가서
구동바퀴와 캐스터가 둘 다 바닥에 닿아 있는지 눈으로 확인하고, 필요하면
`urdf/aircore.xacro`에서 `left/right_support_wheel_joint` 또는
`left/right_wheel_joint`의 `origin xyz` z값을 1~2mm 단위로 조정하세요.

### 9-3-2. (실제 발견된 사례) 로봇이 들썩이면서 아주 느리게 움직임, "기둥/층이
고정 안 된 것 같다" — 바퀴와 차체가 서로 충돌하고 있었음, 지금 고침

`xacro`로 URDF를 SDF로 실제 변환해서(`gz sdf -p`) 내부 구조를 직접
확인했습니다. 결과: `fixed` 조인트로 연결된 링크(floor_2, floor_3,
battery, driver, stmf407, jetson_nano, lidar, support_1_x/support_2_x,
캐스터 2개)는 전부 `base_link` 하나로 합쳐지고, 실제로 물리적으로 분리된
링크는 `base_link`, `left_wheel`, `right_wheel` **딱 3개**뿐이었습니다.

즉 기둥/층 자체가 "안 고정된" 게 아니라 애초에 다른 링크로 존재하지도
않고 전부 base_link에 완전히 하나로 붙어있는 강체입니다 — 그것들끼리
따로 흔들릴 수가 없는 구조입니다. 사용자가 본 "들썩임"은 로봇 전체(하나의
강체)가 통째로 떨리는 걸 보고 그렇게 보인 것이었습니다.

**진짜 원인**: STL 메시 바운딩박스를 확인해보니 `left_wheel`/
`right_wheel`의 메시가 `base_link` 메시와 상당 부분 겹칩니다
(바퀴 y범위 8.2~100mm, z범위 -5.5~59.5mm가 차체 쪽 y/z 범위와 정확히
겹침 — 실제 CAD에서 바퀴 축이 차체 마운트 안쪽으로 들어가 있는 구조라
흔한 일). 그런데 `urdf/aircore.gazebo`에는 **모든 링크에
`<selfCollide>true</selfCollide>`가 걸려 있었습니다** (SolidWorks→URDF
변환기가 자동으로 붙여준 기본값으로 보임). `selfCollide=true`는 "조인트로
직접 연결된 이웃 링크끼리도 충돌 검사를 하겠다"는 뜻이라서, 기본적으로는
무시되는 base_link↔left_wheel/right_wheel 충돌이 매 물리 스텝(1000Hz)마다
계산되고, 겹친 메시를 서로 밀어내려는 힘이 계속 발생합니다. 바퀴가
회전하면서 차체와 계속 부딪히고 밀려나기를 반복하니 명령한 속도로 구르지
못하고 제자리에서 떨면서(들썩이면서) 아주 느리게만 이동하게 됩니다.

**고친 것**: `urdf/aircore.gazebo`의 `<selfCollide>true</selfCollide>`
전부(20곳)를 `false`로 바꿨습니다. 이 로봇은 바퀴 2개를 빼면 전부 고정
조인트라 애초에 내부 자기충돌 검사가 필요 없고, 벽/장애물 등 다른
모델과의 충돌은 이 옵션과 무관하게 그대로 정상 작동합니다.

겸사겸사 `DiffDrive` 플러그인의 `wheel_radius`도 `0.035` → `0.0325`로
고쳤습니다(`left_wheel.stl` 실측 반지름과 일치시킴) — 값이 실제보다 크면
같은 명령 속도에도 로봇이 실제로 이동하는 거리가 약 7% 느리게 계산됩니다.

### 9-3-3. (실제 발견된 사례) Real Time Factor(RTF)가 0.1~0.5로 낮음 — 버그가
아니라 성능 문제, 계산량을 줄여서 완화

사용자가 실제로 Gazebo 상태 표시줄의 RTF를 확인해준 결과 0.1~0.5로 낮게
나왔습니다. **RTF < 1은 "시뮬레이션이 실시간보다 느리게 돈다"는 뜻이고,
이건 URDF/물리 설정이 틀려서가 아니라 이 컴퓨터(GPU 가속 없는 WSL2 등)가
지금 부하를 실시간으로 못 따라가는 성능 문제입니다.** 이 상태에서는
`/cmd_vel`이 정상적으로 나가고 있어도(9-1 확인 완료), 사람이 벽시계
기준으로 보면 로봇이 느리게, 또는 프레임이 뚝뚝 끊기면서(들썩이는 것처럼)
움직이는 것처럼 보입니다. SLAM/Nav2의 정확도 자체는 영향받지 않습니다
(모든 노드가 `use_sim_time`으로 시뮬레이션 시간 기준으로 일관되게 동작
하기 때문) — 다만 지도를 다 그리는 데 걸리는 실제(벽시계) 시간이 더
길어질 뿐입니다.

**줄인 것**:
- `worlds/aircore_world.sdf`: 물리 스텝을 1000Hz(`max_step_size=0.001`) →
  250Hz(`max_step_size=0.004`)로 낮춤. 이 로봇은 바퀴 2개짜리 단순한
  강체 3개 구조라 1000Hz 정밀도가 필요 없습니다.
- `urdf/aircore.gazebo`: `gpu_lidar` 샘플 수 500 → 240, 업데이트 주기
  10Hz → 5Hz로 낮춤. GPU 가속이 없는 환경에서는 라이다의 ogre2 렌더링이
  RTF를 깎아먹는 큰 원인 중 하나인데, slam_toolbox 매핑에는 이 정도
  해상도/주기로도 충분합니다.

이걸 적용해도 RTF가 여전히 많이 낮다면(예: 0.3 미만), 다음으로 시도할 수
있는 것: Gazebo GUI 창을 작게 하거나 최소화(렌더링 부하 감소), 또는
`ros2 launch aircore_description gazebo.launch.py`의 `gz_args`에
`-s`(서버만, GUI 없이 헤드리스)를 추가해서 렌더링 자체를 끄고 물리만
돌리는 방법이 있습니다(다만 이 경우 Gazebo 창으로 눈으로 확인은 못 하고
RViz로만 확인 가능).

### 9-4. 진단에 도움되는 로그
`wander.launch.py`에 진단용 로그를 추가해뒀습니다. 실행 중인 터미널 4에
`front=... left=... right=... -> v=... w=...` 형태로 2초마다 현재 라이다
거리와 실제로 보내는 속도가 찍힙니다. `/scan`이 아예 안 들어오면
`아직 /scan을 한 번도 못 받음...` 경고가 뜨니, 이것만 봐도 원인이
9-1(명령 자체가 없음)인지 9-2/9-3(명령은 있는데 안 움직임)인지 바로
구분됩니다.

### 9-5. (실제 발견된 사례) 지도는 그럴듯하게 그려지는데, 로봇이 계속 같은
자리에 멈춰 있는 것처럼 보임

**증상**: RViz에서 로봇이 회전만 하고 있거나, 같은 위치에서 몇 분째 안
움직이는 것처럼 보임. 지도 자체(구조)는 정상적으로 그려지고 있어서
헷갈릴 수 있음.

**원인**: `wander.py`의 회피 로직은 "회전을 시작하면 앞이 충분히 트일
때까지 유지"하는 방식인데, 코너나 두 벽이 만나는 좁은 틈에 들어가면
회전해도 정면 각도가 조금씩만 바뀌면서 다시 장애물에 걸리는 패턴이
반복되어 **제자리 회전만 계속하고 실제 위치는 거의 안 바뀌는** 경우가
있음. 라이다 값(회전 여부)만 보고 판단하고 오도메트리로 "진짜 이동
거리"를 확인하지 않아서 이 상태를 스스로 감지 못 했음.

**고친 것**: `/odom`을 구독해서 3초마다 실제 이동 거리를 확인하고,
8cm 미만이면 "탈출 기동"(2.5초간 후진 + 무작위 방향으로 크게 회전)을
강제로 실행하도록 `scripts/wander.py`를 수정함.

**확인 방법**: 터미널 4(wander.launch.py) 로그에 `최근 3초간 ...m밖에 안
움직임 -> 탈출 기동 시작`, `탈출 기동 중...` 로그가 뜨는지 확인. 뜬
이후에 로봇이 실제로 다른 곳으로 이동하는지(RViz에서) 확인.

## 10. `/map`이 영원히 안 생기는 문제 — 실제로는 이게 가장 핵심적인 버그였음

**증상**: Gazebo/바퀴/RTF를 다 고친 뒤에도 RViz에서 모든 링크에 대해
`No transform from [X] to [map]` 에러가 뜨고, `ros2 topic echo /map`이
영원히 아무것도 안 받고, `ros2 node info /slam_toolbox`가
"Unable to find node"를 계속 반환함(→ 나중엔 노드가 뜨긴 하는데
`/clock`, `/parameter_events`만 구독하고 있고 **`/scan` 구독도 `/map`
발행도 아예 없음**).

**원인**: `async_slam_toolbox_node`는 평범한 노드가 아니라 **ROS2
LifecycleNode**입니다. 이런 노드는 실행되면 바로 일하는 게 아니라
`unconfigured` 상태로 대기하다가, 외부에서 `configure` → `activate`
전이(transition)를 명시적으로 걸어줘야 그때부터 실제로 토픽을 구독/발행
하기 시작합니다. `launch/slam.launch.py`가 이걸 그냥 평범한 `Node`
액션으로 띄우고 있었어서, 노드는 살아있지만 영원히 `unconfigured`
상태에 멈춰 `/scan`을 구독하지도, `/map`을 만들지도 않고 있었던 겁니다.
`ros2 node info`로 확인한 서비스 목록에 `/slam_toolbox/change_state`,
`/slam_toolbox/get_state` 같은 lifecycle 서비스가 있었던 게 이 노드가
lifecycle 노드라는 결정적 증거였습니다.

실제로 `/opt/ros/jazzy/share/slam_toolbox/launch/online_async_launch.py`
(slam_toolbox 패키지가 공식 제공하는 launch 파일)를 직접 열어서 확인해보니,
`LifecycleNode` 액션으로 띄운 뒤 `EmitEvent(ChangeState(...CONFIGURE))`로
configure시키고, 노드 상태가 `configuring → inactive`로 바뀌는 걸
`OnStateTransition`으로 감지해서 그 시점에 다시
`EmitEvent(ChangeState(...ACTIVATE))`로 activate시키는 구조로 되어
있었습니다. 그동안 겪었던 "Gazebo/바퀴/RTF는 다 고쳤는데 왜 지도가 안
생기지" 문제의 진짜 원인이 바로 이거였습니다.

**고친 것**: `launch/slam.launch.py`를 공식 launch 파일과 동일한 구조로
다시 작성했습니다 — `Node` → `LifecycleNode`로 교체하고, configure/activate
이벤트 핸들러를 추가했습니다. 이제 정상적으로 실행하면 터미널에
`configuring`, `[slam.launch.py] slam_toolbox configure 완료 -> activate
진행`, `activating` 같은 로그가 순서대로 뜨고, 그 이후에 `/scan` 구독과
`/map` 발행이 시작됩니다.

**확인 방법**: `ros2 launch aircore_description slam.launch.py` 실행 후
```bash
ros2 node info /slam_toolbox   # Subscribers에 /scan이 보여야 정상
ros2 topic hz /map             # 몇 초에 한 번씩 찍히면 정상
```

### 10-2. (실제 발견된 사례) `/map`은 잘 나오는데, 같은 방이 회전/이동된 채로
여러 겹 찍혀서 지도가 지저분함

**증상**: `/map`이 정상적으로 발행되고 RViz에도 지도가 그려지긴 하는데,
로봇이 한 바퀴 이상 돌고 나면 같은 방 외곽선(사각형)이 조금씩 회전되거나
어긋난 채로 여러 개 겹쳐서 찍힘. 벽이 하나로 깔끔하게 안 그려지고
같은 벽이 여러 각도로 중첩되어 보임.

**원인 (추정, 확실한 근거는 아직 없음 — 아래 확인 방법으로 검증 필요)**:
`slam_toolbox`는 매 스캔마다 스캔 매칭(scan matching)으로 오도메트리
오차를 보정하는데, 이 보정에 실패하면 그냥 원본(드리프트가 낀)
오도메트리 위치에 스캔을 찍어버립니다. 로봇이 방을 한 바퀴 돌 때마다
추정 방향이 조금씩 틀어지고, 그 틀어진 정도가 `loop_search_maximum_distance`
(3.0m)보다 커지면 slam_toolbox가 "아까 그 방"이라고 재인식(루프 클로저)을
못 해서 매번 새 사본을 그리게 됩니다.

바퀴 반지름(0.0325m)/간격(0.1505m)은 URDF 실측값과 정확히 일치하므로
오도메트리 계산식 자체는 맞습니다. 유력한 후보는 이 환경의 낮은 RTF
(GPU 없음, WSL2 — [9-3-3](#9-3-3) 참고) 때문에 `/tf` 조회가 느려져서
`transform_timeout`(기존 0.2초)을 자꾸 초과하고, 그때마다 스캔 매칭이
스킵/실패해서 보정 안 된 위치로 스캔이 찍히는 것입니다.

**고친 것**: `config/mapper_params_online_async.yaml`의 `transform_timeout`을
`0.2` → `1.0`으로 늘림. 근본 원인이 100% 확인된 건 아니라서, 이걸로 안
고쳐지면 아래 확인 방법으로 추가 진단이 필요합니다.

**확인 방법 (지도를 새로 만들면서 테스트)**:
1. 터미널 3(slam.launch.py)에서 `Ctrl+C`로 종료.
2. `ros2 launch aircore_description slam.launch.py`로 다시 시작 (새 지도로
   초기화됨).
3. `wander.launch.py`로 로봇이 **방을 딱 한 바퀴만** 돌게 하고 지도 확인.
   - 한 바퀴만 돌았을 때 지도가 깔끔하면 → 여러 바퀴 돌 때 드리프트가
     누적되어 루프 클로저가 실패하는 게 맞다는 뜻.
   - 한 바퀴만 돌아도 지저분하면 → 스캔 매칭 자체가 거의 항상 실패하고
     있다는 뜻이므로 라이다 주기(`update_rate`)를 다시 올리거나
     `transform_timeout`을 더 늘리는 등 추가 조치 필요.
4. 터미널 3 로그에서 스캔 매칭 실패/타임아웃 관련 경고 줄이 있는지도 확인.

## 11. 자주 막히는 부분

- **`Package 'aircore_description' not found`**: `~/.bashrc`에 워크스페이스
  오버레이 자동 소스를 추가해뒀습니다. 그래도 나오면 (1) 그 터미널이 이 변경
  전에 이미 열려있던 세션인지(→ `source ~/.bashrc` 한 번 실행), (2) 아직
  `colcon build`를 안 해서 `install/setup.bash`가 없는 상태인지 확인하세요.
- **Gazebo에서 `Unable to find file [model://aircore_description/meshes/...]`
  에러**: urdf의 `package://aircore_description/meshes/...` 경로를 gz-sim이
  `model://aircore_description/...`로 바꿔서 찾는데, `GZ_SIM_RESOURCE_PATH`에
  설치된 share 디렉터리가 없으면 못 찾습니다. `gazebo.launch.py`가 이제
  자동으로 이 환경변수를 설정하도록 고쳤으니, 파일을 다시 받은 뒤
  `colcon build --packages-select aircore_description` 후 재실행하면
  사라집니다. (그래도 남으면 `echo $GZ_SIM_RESOURCE_PATH`로 경로가
  `.../install/aircore_description/share`를 포함하는지 확인)
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
- **Nav2 켰는데 로봇이 안 움직임**: 십중팔구 `wander.launch.py`를 안 끄고 같이
  켜놔서 `/cmd_vel`을 두 노드가 동시에 발행 중인 경우입니다. wander를 끄세요.
  그 다음엔 `ros2 topic echo /cmd_vel`로 Nav2가 값을 보내는지, `ros2 lifecycle
  get /controller_server`로 상태가 `active`인지 확인하세요.
- **`2D Goal Pose`를 찍었는데 경로가 안 나옴/즉시 실패함**: 목적지가 아직 스캔
  안 된(회색/미탐색) 영역이거나 벽 안쪽처럼 코스트맵상 점유된 지점일 수
  있습니다. `GlobalCostmap` 디스플레이를 켜서 실제로 갈 수 있는 흰색/회색
  영역인지 확인하고 다시 찍어보세요.

## 12. 이번 단계에서 일부러 안 한 것

- **STM32 UART 스펙 문서, 실제 장애물 배치/파라미터 튜닝(Day5)** — 계획표상
  이번 범위(Day1~4, SLAM+Nav2) 다음 단계라 제외했습니다.
- **Jetson 실물 연동, AMCL 기반 저장맵 주행** — 지금은 slam_toolbox가 계속
  돌면서 SLAM+로컬라이제이션을 겸하는 구조입니다. 저장한 맵으로 SLAM 없이
  `map_server`+`amcl`만으로 띄우는 구성은 다음 단계로 미뤘습니다.
