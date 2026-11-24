# aircore_ws

> **실차 작업의 현재 상태·미해결 문제·재개 명령은 [`STATUS.md`](STATUS.md)** 에 있다 (2026-11-23 기준).
> 아래 3장의 일부 내용(포트, 측정값)은 그 이전 기록이며, 실차는 `config/real_params.yaml` 을 쓴다.

스마트팜 자율주행 차량 **aircore**의 ROS 2 워크스페이스.
실제 차량 구동(STM32 시리얼 브리지 + 오도메트리)과 Gazebo 시뮬레이션(맵 + 차량)을 담고 있다.

- OS / ROS: Ubuntu 22.04.5, ROS 2 Humble
- 컴퓨터: NVIDIA Jetson Orin Nano (8 GB, CPU·GPU 메모리 공유)
- 시뮬레이터: **Gazebo Fortress** (`ign gazebo` 6.18). Harmonic(`gz sim`)은 사용하지 않는다 (아래 [6. 환경 메모](#6-환경-메모) 참고)

---

## 1. 패키지 구성

| 패키지 | 종류 | 역할 |
|---|---|---|
| `aircore_base` | ament_python | 실제 차량: STM32 시리얼 브리지 + 모터 제어(`base_controller`), 바퀴 오도메트리(`odometry`) |
| `aircore_sim` | ament_python | 시뮬레이션: STL 맵 → Gazebo 월드, Nav2용 2D 맵, 차량 모델 삽입 |
| `aircore_description` | ament_cmake | 차량 URDF + 메시 (`~/gazebo_map_ws/aircore_description`에서 복사, 원본은 그대로 둠) |

```
src/
├── aircore_base/
│   ├── aircore_base/base_controller.py   # 기존 (수정 금지)
│   ├── aircore_base/odometry.py          # 추가
│   ├── config/base_params.yaml
│   └── launch/base.launch.py             # base_controller + odometry
├── aircore_description/
│   ├── urdf/aircore.urdf
│   ├── meshes/*.stl
│   └── launch/display.launch.py          # RViz 확인용
└── aircore_sim/
    ├── aircore_sim/farm_geometry.py      # 맵 기본값·STL 분석·SDF 생성 (맵 관련 값은 여기 한 곳)
    ├── aircore_sim/vehicle.py            # URDF → Fortress 차량 모델, 스폰 위치 기본값
    ├── launch/sim_world.launch.py
    ├── scripts/generate_world.py         # 패키지 안의 월드/모델 파일 재생성
    ├── scripts/make_occupancy_map.py     # 2D 점유 격자 맵 생성
    ├── models/farm_map/                  # model.sdf, model.config, meshes/MAP_3.stl
    ├── worlds/farm.sdf                   # 맵만 있는 월드 (차량 없음)
    └── maps/farm.pgm, farm.yaml, farm_preview.png
```

## 2. 빌드

```bash
cd ~/aircore_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 3. aircore_base (실제 차량)

### 3.1 base_controller (기존 코드, 수정하지 않음)
- STM32와 시리얼 통신 (기본 `/dev/ttyUSB0`, 115200)
  - Jetson → STM32: `M,left,right\n` (duty, -100~100)
  - STM32 → Jetson: `ENC,left,right\n` (누적 엔코더 카운트, 50 ms 주기)
- STM32는 UART가 300 ms 이상 조용하면 모터를 멈추므로 20 Hz로 계속 송신(keep-alive)
- `/cmd_vel` 구독 → duty 변환, `/aircore/enc_raw` (std_msgs/Int32MultiArray, signed int32) 발행
- 안전 장치: `wheel_separation`/`max_wheel_speed` 미설정(0)이면 `M,0,0`만 송신, `max_duty: 30`, `cmd_vel_timeout: 0.5 s`, `dry_run`

### 3.2 odometry (추가)
`/aircore/enc_raw` → `/odom` (nav_msgs/Odometry) + TF `odom → base_link`

- 좌우 카운트 차이를 이전 값과 비교해서 계산, **int32 wraparound 처리** (차이 ≥ 2³¹ 이면 −2³², < −2³¹ 이면 +2³²)
- `d = dn / ticks_per_rev × 2π r`, `d = (dR + dL)/2`, `dθ = (dR − dL)/L`, **중점법 적분**
- 속도는 메시지 수신 시각 차이(노드 clock)로 계산. 첫 메시지는 기준값만 저장
- covariance: pose 대각 `[0.01, 0.01, 1e6, 1e6, 1e6, 0.03]`, twist 대각 `[0.01, 1e6, 1e6, 1e6, 1e6, 0.03]`
- **측정값 4개 중 하나라도 0이면 `/odom`, TF를 내보내지 않고 warn을 한 번만 출력**

| 파라미터 (`config/real_params.yaml` 의 `odometry:`, 2026-11-23) | 현재 값 | 상태 |
|---|---|---|
| `wheel_radius` | 0.0325 | 실측 (지름 65 mm) |
| `wheel_separation` | 0.217 | CAD 값 (URDF), 실차 미확인 |
| `ticks_per_rev_left` / `_right` | 687 / 685 | `measure_ticks.py` 실측 |
| `invert_enc_left` | true | 확정 |
| `invert_enc_right` | true | 2026-11-24 false → true (오른쪽 배선 손본 뒤 부호 바뀜, STATUS.md 참고) |
| `odom_frame` / `base_frame` / `publish_tf` | odom / base_footprint / true | URDF 루트에 맞춤 |

(`config/base_params.yaml` 은 원본 그대로 0 값과 `/dev/ttyUSB0` 을 유지한다.)

> `base_controller:` 섹션에도 `wheel_separation`, `wheel_radius`가 따로 있다. 실측 후에는 **두 섹션 모두** 넣어야 한다.

### 3.3 실행
```bash
ros2 launch aircore_base base.launch.py              # base_controller + odometry
ros2 launch aircore_base base.launch.py dry_run:=true  # 시리얼에 쓰지 않고 로그만
```
⚠ `base_controller`는 시리얼 포트를 열고 모터 명령을 보낸다. 바퀴를 띄운 상태에서 실행할 것.

⚠ 위 `base.launch.py` 기본값(`base_params.yaml`)의 `/dev/ttyUSB0` 은 현재 **LiDAR 포트**다. 실차는
`bringup.launch.py` / `odom_view.launch.py` (기본값 `real_params.yaml`, by-id 경로)를 쓴다. 사용법은 `STATUS.md`.

---

## 4. aircore_sim (시뮬레이션)

### 4.1 맵 (MAP_3.stl)
원본: `~/Desktop/MAP_3.stl` → `models/farm_map/meshes/MAP_3.stl`로 복사 (원본 수정 안 함, md5 동일 확인)

STL 분석 결과 (원본 단위 **mm 확정**, `map_scale = 0.001`):

| 항목 | STL (mm) | 월드 (m) |
|---|---|---|
| 바이너리 STL, 삼각형 | 108개 | |
| bbox | x −250~550, y 0~1200, z 0~55 | |
| 바닥판 | 800 × 1200, 두께 5 | 0.8 × 1.2, 두께 0.005 |
| 긴 블록 | x −200~−120, y 120~1150, z 5~45 | 폭 0.08, 높이 **0.04** |
| 작은 블록 5개 | x 100~180, 길이 120, 간격 40, z 5~55 | 폭 0.08, 길이 0.12, 높이 **0.05** |
| 긴 블록 ~ 작은 블록 통로 | 220 | **0.22** |

### 4.2 월드 구성
- **바닥 x폭 1.6 m로 확장** (`FLOOR_WIDTH_X = 1.6`): STL은 그대로 두고, 같은 두께(5 mm) 바닥 박스를 STL 바닥판 **+x쪽(빈 공간 쪽)** 에 덧붙임. 블록 크기·배치는 그대로
- **좌표계**: 바닥 전체의 중심 = 월드 원점, 바닥 윗면 = z 0
  - 바닥: x −0.8~0.8, y −0.6~0.6 (1.6 × 1.2 m)
  - 긴 블록 x −0.75~−0.67 / 작은 블록 x −0.45~−0.37 / 작은 블록 오른쪽 빈 공간 1.16 m
  - STL mesh pose 오프셋: (−0.55, −0.6, −0.005)
- **4면 벽**: 바닥판 가장자리 안쪽에 박스 4개 (바깥면이 바닥 가장자리와 일치). `wall_height 0.10 m`, `wall_thickness 0.01 m` 은 **임시값**
- 지면(ground plane)은 바닥판 밑면 높이(z = −0.005)에 둠. z = 0에 두면 바닥판 윗면과 같은 높이라 화면이 깜빡임(z-fighting)

**기본값은 한 곳에서만 바꾼다**
- `aircore_sim/farm_geometry.py`: `MAP_SCALE`, `FLOOR_WIDTH_X`, `WALL_HEIGHT`, `WALL_THICKNESS`, `SENSOR_RENDER_ENGINE`
- `aircore_sim/vehicle.py`: `SPAWN_X`, `SPAWN_Y`, `SPAWN_YAW`

launch는 실행할 때마다 이 값과 launch 인자로 `~/.ros/aircore_sim/generated/`에 월드를 새로 만든다.
패키지 안의 `models/farm_map/model.sdf`, `worlds/farm.sdf`, `maps/*`는 기본값을 바꾼 뒤 아래 스크립트를 다시 실행해야 갱신된다.

### 4.3 차량 (aircore)
- 원본 URDF: `aircore_description/urdf/aircore.urdf` (2026-10 버전)
  - 0.230 × 0.243 × 0.204 m, 구동바퀴 간격 0.217 m, 반지름 0.0325 m, 볼캐스터 2개, RPLIDAR C1 (`gpu_lidar`, 500 샘플, 10 Hz)
- launch 때 `ign sdf -p`(Fortress sdformat 12)로 변환하고 Fortress용으로 두 가지만 바꾼다
  - 구동 플러그인: `ignition-gazebo-diff-drive-system`, `ignition-gazebo-joint-state-publisher-system`
  - 라이다 frame: Harmonic `gz_frame_id` → Fortress `ignition_frame_id`
- 시뮬레이터 DiffDrive의 바퀴 간격·반지름은 **URDF(CAD) 값**이다. 실측값이 아니라서 `aircore_base` 파라미터에는 넣지 않았다
- 기본 스폰 위치 (0.2, 0.0, yaw 0): 작은 블록 오른쪽 빈 공간. 블록·벽과 겹치거나 바닥 밖이면 launch가 warn 출력 (차량 반경 0.170 m, 충돌 형상 기준으로 넉넉하게 잡은 값)

Gazebo 토픽 (**아직 ROS 브리지 없음**. 실제 로봇의 `/cmd_vel`, `/odom`과 섞이지 않음):

| 토픽 | 내용 |
|---|---|
| `/model/aircore/cmd_vel` | 주행 명령 (ignition.msgs.Twist) |
| `/model/aircore/odometry`, `/model/aircore/tf` | 오도메트리 (odom → base_footprint) |
| `/scan`, `/scan/points` | 라이다 |
| `/world/farm/model/aircore/joint_state` | 바퀴 관절 상태 |

### 4.4 실행
```bash
ros2 launch aircore_sim sim_world.launch.py                     # GUI + 맵 + 차량
ros2 launch aircore_sim sim_world.launch.py headless:=true      # GUI 없이 서버만
ros2 launch aircore_sim sim_world.launch.py with_vehicle:=false # 맵만
ros2 launch aircore_sim sim_world.launch.py spawn_x:=0.4 spawn_y:=-0.3 spawn_yaw:=1.57

# 주행 (다른 터미널)
ign topic -t /model/aircore/cmd_vel -m ignition.msgs.Twist -p 'linear: {x: 0.1}'
ign topic -t /model/aircore/cmd_vel -m ignition.msgs.Twist -p 'linear: {x: 0.0}'
```
launch 인자: `map_scale`, `floor_width_x`, `wall_height`, `wall_thickness`, `with_vehicle`, `spawn_x`, `spawn_y`, `spawn_yaw`, `headless`

맵만 있는 월드를 launch 없이 직접 실행:
```bash
cd ~/aircore_ws/install/aircore_sim/share/aircore_sim
IGN_GAZEBO_RESOURCE_PATH=$PWD/models ign gazebo -s -r --iterations 10 worlds/farm.sdf
```

### 4.5 Nav2용 2D 맵
```bash
cd ~/aircore_ws/src/aircore_sim
python3 scripts/make_occupancy_map.py                      # 기본값
python3 scripts/make_occupancy_map.py --scan-height 0.195  # 예: 라이다 높이에 맞춤
python3 scripts/generate_world.py                          # 패키지 안의 월드/모델 파일 재생성
```
- SLAM 없이 STL 박스와 벽 파라미터에서 직접 계산
- 출력: `maps/farm.pgm`, `maps/farm.yaml` (Nav2 map_server 형식, `negate 0`, `occupied_thresh 0.65`, `free_thresh 0.25`)
- 현재 맵: **160 × 120칸 × 0.01 m = 1.6 × 1.2 m**, origin (−0.8, −0.6)
- 점유 규칙: 바닥 높이 기준 `zmin ≤ scan_height < zmax` 인 장애물(블록, 벽) → 점유(0), 바닥판 밖 → 점유(0), 나머지 → free(254)
- 인자: `--resolution` (기본 0.01), `--scan-height` (기본 0.02, **임시값**), `--margin` (바닥 밖 여백, 점유로 채움), `--floor-width-x`, `--wall-height`, `--wall-thickness`
- 미리보기: `src/aircore_sim/maps/farm_preview.png` (검정 = 바닥 밖, 파랑 = 벽, 초록 = 블록, 빨간 십자 = 원점, 격자 0.1 m)

---

## 5. 테스트 기록

모든 테스트는 하드웨어 없이 진행했다 (`/dev/ttyUSB*` 접근 없음, `base_controller` 실행 안 함).
다른 ROS/Gazebo 통신과 섞이지 않게 `ROS_DOMAIN_ID=87`, `ROS_LOCALHOST_ONLY=1`, `IGN_PARTITION=aircore_test` 로 분리했다.

### odometry (2026-11-21)
테스트용 파라미터: r = 0.03, L = 0.2, ticks = 440, invert 둘 다 false. `ros2 topic pub`으로 가짜 엔코더 값 입력.

| 항목 | 입력 | 결과 |
|---|---|---|
| 직진 | 좌우 0 → +440 | x = 0.188496 (기대 0.1885) ✅ |
| 누적 | +440씩 3회 | x 0.1885 → 0.3770 → 0.5655 ✅ |
| 제자리 회전 | 좌 −440, 우 +440 | x = y = 0, θ = 1.884956 ✅ |
| int32 경계 | 2147483647 → −2147483648 | 차이 +1 (x = 0.000428) ✅, 역방향 −1 ✅ |
| `invert_enc_left=true` | 좌 −440, 우 +440 | 직진 0.1885 ✅ |
| 미설정(0) | 메시지 4개 | warn 1회, `/odom`·TF 0건 ✅ |

테스트 후 yaml은 미설정 값으로 되돌림.

### 시뮬레이션 맵 (2026-11-21)
- 헤드리스 `ign gazebo -s -r --iterations 10`: 오류·경고 0 (`-v 3`, `-v 4`)
- 반지름 0.01 m 공 낙하로 충돌 확인: 블록 위 0.060, 바닥판 0.010, STL 바닥판과 확장 바닥의 이음매 0.010 (틈 없음), 동쪽 벽 위 0.110, 바닥 밖 지면 0.005 ✅
- 2D 맵: 크기·origin 확인, 통로 0.22 m free, 벽 테두리 전부 점유, Nav2 `map_server` 로드 시 경고 없음 ✅

### 차량 (2026-11-21~22)
- 헤드리스: 오류·경고 0, 주행 0.1 m/s 약 2.5초에 x 0.20 → 0.45 직진 ✅
- 라이다: 500 샘플 정상. 높이 0.3 m 테스트 상자를 앞 0.33 m에서 검출 ✅
- GUI: 차량, 맵, 벽 정상 표시, 실시간 비율 약 0.86~1.0 ✅

---

## 6. 환경 메모

- **Fortress와 Harmonic이 둘 다 설치돼 있다.** Harmonic(`gz-harmonic`)은 2026-11-19에, Fortress(`ignition-fortress`, `ros-humble-ros-gz`)는 2026-11-21에 설치. Humble의 `ros-humble-ros-gz`는 Fortress 기준이라 **이 워크스페이스는 `ign` 명령만 쓴다**. `~/gazebo_map_ws`는 Harmonic용으로 만든 이전 작업이다
- **lidar 렌더링은 ogre1** (`SENSOR_RENDER_ENGINE = 'ogre'`): ogre2로 두면 서버의 lidar 렌더링과 GUI(ogre2)가 함께 GPU 메모리를 써서, Orin Nano에서 `NvMapMemAllocInternalTagged ... error 12`가 나고 3D 화면이 검게 된다. 다른 프로그램(브라우저 등)이 메모리를 많이 쓰면 다시 생길 수 있다
- 실행 시 `libEGL warning: ... nvidia-drm_dri.so ... failed to create dri2 screen` 은 Jetson에서 Mesa가 드라이버를 먼저 찾다가 실패할 때 나오는 메시지로, 렌더링에는 영향이 없었다
- Gazebo GUI에서 맵이 작게 보이면 카메라를 맵으로 옮기면 된다:
  `ign service -s /gui/move_to --reqtype ignition.msgs.StringMsg --reptype ignition.msgs.Boolean --timeout 3000 --req 'data: "farm_map"'`

---

## 7. 미결 / 임시값 (TODO)

| 항목 | 현재 | 해야 할 일 |
|---|---|---|
| ~~STM32 보드~~ | 교체 완료 (2026-11-23) | 통신 확인 완료 |
| ~~오도메트리 측정값~~ | `real_params.yaml` 에 입력 완료 | `wheel_separation` 만 CAD 값 → 바닥 360° 회전으로 확인 |
| **오른쪽 모터** | duty 20 에서 회전 없음 | 배선/드라이버/펌웨어 점검 (자세한 내용 `STATUS.md`) |
| `max_wheel_speed`, `invert_right` | 미측정 / 미확인 | 오른쪽 모터 해결 후 `motor_test.py --sweep` |
| 시뮬레이션 라이다 방향 | URDF `laser_joint` yaw = π 로 바뀜 | 시뮬레이션에서 재확인 |
| 벽 높이 / 두께 | 0.10 / 0.01 m (임시) | 실측 후 `farm_geometry.py` 수정, 월드·맵 재생성 |
| `scan_height` | 0.02 m (임시) | 라이다 실제 높이로 설정 |
| **라이다 높이 문제** | 라이다 0.195 m (URDF) > 벽 0.10 m, 블록 0.04~0.05 m | 지금은 시뮬레이션에서 라이다가 아무것도 못 봄. 실제 벽 높이 확인 필요. `scan_height`를 0.195로 맞추면 현재 벽·블록은 2D 맵에서 모두 빠짐 |
| **통로 폭 문제** | 차량 폭 0.243 m > 통로 0.22 m | 차량은 통로를 못 지나감. 오른쪽 빈 공간(1.16 m)에서만 주행 가능 |
| ROS ↔ Gazebo 브리지 | 없음 | `ros_gz_bridge`로 cmd_vel, odom, scan, clock, tf 연결 (실제 로봇 토픽과 이름 충돌 주의) |

---

## 8. 작업 이력

| 날짜 | 작업 |
|---|---|
| 2026-11-21 | USB-TTL(FT232 `/dev/ttyUSB0`, CP2102 `/dev/ttyUSB1`) ↔ STM32F407 통신 점검. 두 포트 모두 수신 0바이트 (이후 STM32 보드 고장 확인) |
| 2026-11-21 | `aircore_base/odometry.py` 추가, 가짜 엔코더 값으로 테스트 |
| 2026-11-21 | `aircore_sim` 생성: STL 맵 검증, Fortress 월드, 4면 벽, 2D 맵 생성 스크립트 |
| 2026-11-21 | Gazebo Fortress + ros_gz 설치 후 헤드리스·GUI 테스트 |
| 2026-11-21 | 바닥 x폭 0.8 → 1.6 m (+x쪽 확장, 블록 그대로), 원점을 새 바닥 중심으로 |
| 2026-11-21~22 | `aircore_description` 복사, 차량 모델 삽입 (Fortress 변환), lidar 렌더링 ogre1로 변경 |
| 2026-11-23 | 새 STM32 보드 실차 연결 (FTDI by-id), ticks 측정·r/L 적용, `/odom` + RViz 확인 |
| 2026-11-23 | `sllidar_ros2` 빌드, `bringup.launch.py` (URDF TF + C1), C1 180° 장착 → `laser_joint` yaw π |
| 2026-11-23 | `motor_test.py` 로 1단계 시작: 왼쪽 `invert_left: true`, **오른쪽 모터 회전 없음 (미해결)** |
