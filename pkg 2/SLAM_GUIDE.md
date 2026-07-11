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
| `urdf/aircore.gazebo` | 라이다: `ray` → `gpu_lidar` 센서로 변경. 바퀴 구동: `libgazebo_ros_diff_drive.so` → `gz::sim::systems::DiffDrive` 플러그인으로 교체 (바퀴 간격도 실제 조인트 좌표 기준 0.1505m로 보정, 기존 값 0.2는 오차 있었음). `JointStatePublisher` 플러그인 추가 |
| `worlds/aircore_world.sdf` | 새로 생성 → 두 번 갱신. 바닥/조명 + gz-sim 필수 시스템 플러그인 + **7m×7m 방을 십자 칸막이로 4개 소방(NE/NW/SE/SW)으로 분리, 각 방마다 다른 모양 장애물(박스/기둥/L자 벽) 2개씩** — 처음엔 빈 바닥에 박스 1개뿐이라 맵이 너무 단순했던 걸 구조물 많은 난이도로 교체 |
| `scripts/wander.py` | 새로 생성 → 갱신. `/scan` 기반 단순 반응형 자동 주행(장애물 회피) 노드 — 직접 조종 안 해도 로봇이 알아서 돌아다니게 함. `/scan` 끊김 감시(watchdog)와 속도 로그를 추가해서 "안 움직임" 문제 진단이 쉬워지게 함 |
| `launch/wander.launch.py` | 새로 생성. `wander.py` 실행용 |
| `config/bridge.yaml` | 새로 생성. gz ↔ ROS2 토픽 브릿지 설정 (`/cmd_vel`, `/odom`, `/tf`, `/scan`, `/joint_states`, `/clock`) |
| `config/mapper_params_online_async.yaml` | 새로 생성. slam_toolbox 온라인 매핑 파라미터 |
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

### 9-3. 바퀴는 도는데 로봇 자체(차체)가 안 움직이나? ★가장 유력한 원인
**이 증상(Gazebo 정지 + RViz만 이동)의 가장 흔한 진짜 원인입니다.**

gz-sim의 `DiffDrive` 플러그인이 계산하는 `/odom`은 실제 물리 위치가 아니라
**바퀴가 얼마나 돌았는지(휠 회전량)로 역산한 추정값**입니다. 즉 바퀴가
헛돌기만 해도(바닥에 제대로 접지가 안 돼서 미끄러지거나, 아예 살짝 떠
있어서 마찰이 안 걸리면) `/odom`은 "전진했다"고 계산해버리고, 그 odom을
그대로 받아쓰는 RViz/slam_toolbox 상에서는 로봇이 이동한 것처럼 보이는데,
Gazebo의 실제 렌더링(진짜 물리 위치)은 그대로인 겁니다.

이 로봇의 `urdf/aircore.gazebo`를 보면 캐스터(`left/right_support_wheel`)가
진짜 자유회전 볼이 아니라 **낮은 마찰(mu=0.01)의 고정 조인트**로 근사돼
있고, `README.md`에도 이 부분이 "검토 후 필요하면 수정" 대상으로 명시돼
있습니다. 캐스터와 구동바퀴의 바닥 접지 높이가 설계값과 조금만 달라도
(캐스터가 너무 낮아서 로봇 무게를 다 받고 구동바퀴는 붕 뜨는 경우 등)
정확히 이 증상이 납니다.

**확인 방법**: Gazebo 창에서 카메라를 로봇 바로 옆(측면, 바닥 높이)으로
가까이 가져가서, 구동바퀴와 캐스터가 둘 다 바닥에 닿아 있는지 눈으로
확인하세요. 구동바퀴가 바닥에서 살짝 떠 있거나 로봇이 캐스터 쪽으로
기울어 있으면 이게 원인입니다.

**고치는 방법** (원인이 이거라고 확인되면): `urdf/aircore.xacro`에서
`left_support_wheel_joint`/`right_support_wheel_joint`의 `origin xyz`
z값을 지금보다 위로(양수 방향으로 2~5mm) 조금 올려서 캐스터가 바닥에 안
닿게(또는 아주 살짝만 닿게) 만들어보세요. 반대로 구동바퀴 쪽이 뜬 거라면
`left_wheel_joint`/`right_wheel_joint`의 z를 낮추는 것도 방법입니다.
수정 후 `colcon build` 하고 Gazebo를 다시 스폰해서 확인하세요.

### 9-4. 진단에 도움되는 로그
`wander.launch.py`에 진단용 로그를 추가해뒀습니다. 실행 중인 터미널 4에
`front=... left=... right=... -> v=... w=...` 형태로 2초마다 현재 라이다
거리와 실제로 보내는 속도가 찍힙니다. `/scan`이 아예 안 들어오면
`아직 /scan을 한 번도 못 받음...` 경고가 뜨니, 이것만 봐도 원인이
9-1(명령 자체가 없음)인지 9-2/9-3(명령은 있는데 안 움직임)인지 바로
구분됩니다.

## 10. 자주 막히는 부분

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

## 11. 이번 단계에서 일부러 안 한 것

- **STM32 UART 스펙 문서, 실제 장애물 배치/파라미터 튜닝(Day5)** — 계획표상
  이번 범위(Day1~4, SLAM+Nav2) 다음 단계라 제외했습니다.
- **Jetson 실물 연동, AMCL 기반 저장맵 주행** — 지금은 slam_toolbox가 계속
  돌면서 SLAM+로컬라이제이션을 겸하는 구조입니다. 저장한 맵으로 SLAM 없이
  `map_server`+`amcl`만으로 띄우는 구성은 다음 단계로 미뤘습니다.
