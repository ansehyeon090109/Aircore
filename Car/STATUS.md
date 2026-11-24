# aircore 실차 작업 현황 (마지막 갱신: 2026-11-24)

다시 시작할 때 이 파일부터 읽는다. 배경 설명과 시뮬레이션은 `README.md`.

## 한눈에

| 단계 | 상태 |
|---|---|
| STM32 통신 (새 보드) | ✅ 완료. ENC 약 20 Hz 수신, `M,0,0` 송신 |
| odometry 실측·적용 | ✅ 완료. `/odom` 20 Hz, RViz 확인 |
| LiDAR(C1) + URDF TF + RViz | ✅ 완료. `/scan` 10 Hz, 앞뒤·좌우 방향 확인 |
| 1단계: 모터 방향 + `max_wheel_speed` | ✅ 완료 (2026-11-24). `max_wheel_speed` 2.65, `invert_enc_right` true, 확인 sweep 에서 두 바퀴 모두 전진 |
| **2단계: 바닥 주행 보정** | **진행 중. 바닥에서 출력 부족·좌우 차이·주행 중 정지 → 원인 확인 중 (아래 "로그 분석")** |
| 3단계: SLAM (slam_toolbox) → Nav2 | 대기 (패키지는 설치돼 있음) |

- ⚠ **`max_wheel_speed: 2.65` 적용됨 (사용자 승인 2026-11-24) → bringup 을 켜고 `/cmd_vel` 을 보내면 실제로 주행한다.** (`max_duty: 100`, 11-24 사용자 지시로 30 → 60 → 100 = **소프트웨어 상한 없음**. `/cmd_vel` 2.65 m/s 면 duty 100) 다시 막으려면 0.0 으로.
- 2026-11-24 20:40 기준 사용자가 띄운 bringup (20:03 시작) 실행 중.

## 다시 시작할 때 명령

모든 터미널에서 먼저:
```bash
source /opt/ros/humble/setup.bash
source ~/aircore_ws/install/local_setup.bash   # 새 패키지를 빌드했으면 이미 연 터미널도 다시 source
export ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1
```

| 하려는 것 | 명령 |
|---|---|
| 전체 (base + odom + URDF TF + LiDAR + RViz) | `ros2 launch aircore_base bringup.launch.py` |
| RViz 없이 / LiDAR 없이 / STM32 없이 | `... rviz:=false` / `lidar:=false` / `base:=false` |
| base + odom 만 (RViz는 odom 전용) | `ros2 launch aircore_base odom_view.launch.py [rviz:=false]` |
| 모터 테스트 (**bringup 끄고**, 바퀴 띄우고) | `ros2 run aircore_base motor_test.py --sweep 20,25,30` 또는 `--left 20` / `--right 20` (`yes` 입력 요구) |
| 실측값 적용 (r, L, ticks) | `ros2 run aircore_base apply_measurements.py --wheel-radius .. --wheel-separation .. --ticks-left .. --ticks-right ..` |
| ticks 재측정 | `ros2 run aircore_base measure_ticks.py` (bringup/odom_view 실행 중에) |

⚠ `ros2 launch aircore_base base.launch.py` 를 인자 없이 쓰지 말 것: 기본값 `base_params.yaml` 의 `serial_port: /dev/ttyUSB0` 은 **연결 순서에 따라 LiDAR 일 수도 STM32 일 수도 있다** (11-23 엔 LiDAR, 11-24 엔 STM32). 실차는 항상 `real_params.yaml` (위 launch 들의 기본값)을 쓴다.

## 하드웨어 / 포트

| 장치 | USB-TTL | 경로 (항상 by-id 사용) |
|---|---|---|
| STM32 (모터 + 엔코더) | FTDI FT232R | `/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AZ5HR0DB-if00-port0` (ttyUSB 번호는 바뀜: 11-23 ttyUSB1, 11-24 ttyUSB0) |
| RPLIDAR C1 | CP2102N | `/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_f03967c3351ef111b3a0c8e40f0f12f8-if00-port0` (11-23 ttyUSB0, 11-24 ttyUSB1) |

- 사용자 `system` 은 `dialout` 그룹 (sudo 불필요)
- 프로토콜: Jetson → STM32 `M,left,right\n` (duty −100~100), STM32 → Jetson `ENC,left,right\n` (50 ms). STM32 는 UART 300 ms 무통신 시 모터 정지
- 연결 직후 첫 줄 `'\x00'` malformed 경고가 한 번 나오는 건 무해

## 현재 파라미터 (`src/aircore_base/config/real_params.yaml`)

| 키 | 값 | 출처 / 상태 |
|---|---|---|
| `wheel_radius` (두 섹션) | 0.0325 m | 실측 (지름 65 mm) |
| `wheel_separation` (두 섹션) | 0.217 m | **CAD 값 (aircore.urdf), 실차 미확인** → 2단계에서 바닥 360° 회전으로 확인 |
| `ticks_per_rev_left` / `right` | 687 / 685 | `measure_ticks.py` 5바퀴 측정 (좌우 차이 0.2%) |
| `invert_enc_left` | true | 확정 (11-23 손으로 굴려 확인, 11-24 sweep 육안과도 일치) |
| `invert_enc_right` | **true** | **2026-11-24 false → true 로 변경**: sweep 에서 바퀴는 육안으로 전진인데 엔코더 카운트 감소. 11-23 손 굴림 땐 false 로 정상이었음 → 오른쪽 배선 손보면서 엔코더 A/B 가 바뀐 것으로 추정. 확인용 sweep 남음 |
| `invert_left` (모터) | **true** | 11-23 `motor_test.py` (M,+20,0 에서 엔코더 기준 후진), 11-24 sweep 육안 전진 확인 |
| `invert_right` (모터) | false | 11-24 sweep (`M,-20,20`) 에서 사용자 육안 확인: 오른쪽 전진 |
| `max_wheel_speed` | **2.65 m/s** | 11-24 sweep 원점 통과 근사 (L 2.59, R 2.70, 차이 4%, 바퀴 띄운 무부하). 사용자 승인 후 적용. 2단계에서 재보정 |
| `max_duty` | **100** | 11-24 사용자 지시로 30 → 60 → 100 (바닥에서 duty 23 도 거의 안 움직여서). 사실상 상한 없음. 바꾸면 bringup 재시작 필요 |
| odometry `base_frame` | base_footprint | URDF 루트에 맞춤 (`odom → base_footprint → base_link → laser`) |
| `sllidar_node` | CP2102N by-id, 460800, frame `laser`, Standard | |

원본 `config/base_params.yaml` 은 손대지 않음 (예전 값 그대로).

2026-11-24 dry_run 검증 (임시값 L=0.3, max_wheel_speed=1.0, max_duty=30 은 scratchpad 에만, `real_params.yaml` 변경 없음):
invert=(true,false) 에서 전진 0.2 → `M,-20,20`, 회전 ω=1.0 → `M,15,15`, 후진 −0.15 → `M,15,-15`. 코드 순서 = 듀티 계산 → ±max_duty 클램프 → 부호 반전 (정상).

## ✅ 해결: 오른쪽 모터가 돌지 않던 문제 (2026-11-24)

2026-11-24 사용자가 터미널에서 STM32 에 raw `M,20,20` 을 직접 보내 **양쪽 모터 회전 확인**. 원인/조치 내용은 기록 없음.
이때 왼쪽이 반대로 돈 것은 **정상**: raw 명령은 `invert_left` 를 거치지 않는다. base_controller·motor_test.py 는 전진일 때 왼쪽에 음수를 보낸다 (`M,-20,20`).
→ `invert_left: true` 그대로 둠 (2026-11-23 엔코더 기준으로 확정한 값).

아래는 2026-11-23 기록 (바퀴 띄운 상태, `motor_test.py` 결과):

| 시험 | 결과 |
|---|---|
| 왼쪽 duty 15, 2 s | 회전 없음 (정지마찰 이하) |
| 왼쪽 duty 20, 2 s | −1.45 rev, 1.00 rev/s = 0.204 m/s (무부하), 방향 반대 → `invert_left: true` 적용 |
| 오른쪽 duty 20, 2 s | **회전 없음, 엔코더 정확히 0** (왼쪽 명령 때 오른쪽 0 → 좌우 매핑은 정상) |
| 오른쪽 duty 30 | Claude 실행이 auto mode 권한 분류기에 막혀 **안 함** |

- 오른쪽 엔코더는 손으로 굴릴 때 정상(685/rev)이었으므로, 모터가 실제로 안 돈 것으로 판단
- 원인 후보: ① 오른쪽 모터 커넥터/배선 ② 모터 드라이버 오른쪽 채널 (enable/STBY, 전원) ③ STM32 펌웨어 오른쪽 PWM 핀 설정 (펌웨어 소스는 이 Jetson 에 없음) ④ 오른쪽만 정지마찰 큼 (가능성 낮음)
- 아직 모르는 것: 시험 중 오른쪽 모터에서 소리가 났는지 (소리 有 → ④/전원, 無 → ①~③)

### 2026-11-24 sweep 결과 (바퀴 띄움, `motor_test.py --sweep 20,25,30`, 설정 변경 전)

| duty | 왼쪽 m/s | 오른쪽 m/s (엔코더 부호 그대로) |
|---|---|---|
| 20 | +0.372 | −0.386 |
| 25 | +0.649 | −0.647 |
| 30 | +0.873 | −0.936 |

- 오른쪽 음수 = 엔코더 부호 문제였음 (육안으로는 두 바퀴 모두 전진) → `invert_enc_right: true` 로 해결
- 속도가 duty 에 비례하지 않음: 직선 근사하면 duty 약 13 이하에서는 거의 안 돎 (dead band). base_controller 는 원점 통과 비례 모델이라
  `/cmd_vel` 이 작으면 바퀴가 안 움직인다: 전진 0.3 m/s → duty 11, 0.5 → 19, **제자리 회전 1.0 rad/s → duty 4 (안 돎)**
- 11-23 왼쪽 duty 20 은 0.204 m/s 였는데 오늘은 0.372 m/s → 배터리/전원 상태에 따라 속도가 크게 바뀜

### 2026-11-24 확인 sweep (설정 변경 후)
- 두 바퀴 모두 전진 속도로 측정됨 (스크립트가 양쪽 값을 계산 = 양쪽 다 엔코더 기준 전진) → 방향 설정 확정
- `max_wheel_speed` 근사: 왼쪽 3.030, 오른쪽 2.804, 평균 2.917 (좌우 차이 7.8%)
- 첫 sweep (2.65) 과 10% 차이, 특히 왼쪽이 2.59 → 3.03. 무부하 측정은 회차마다 ±10% 흔들림 → **2.65 그대로 둠** (바닥에서 재보정)
- 좌우 속도 차이가 회차마다 4~8% → base_controller 는 개루프(엔코더 피드백 없음)라 직진 시 한쪽으로 휠 수 있음. 2단계에서 확인

## 2단계: 바닥 주행 보정 (2026-11-24 시작)

도구: `scripts/floor_calib.py` (teleop 대신). bringup 실행 중에 `/cmd_vel` 로 정해진 직진/제자리 회전 → odom 목표에서 정지 →
LiDAR 스캔 정합(ICP)으로 실제 이동 측정 + 줄자/눈 값 입력 → `wheel_radius` / `wheel_separation` 제안 (적용은 `apply_measurements.py`).
- 안전: 계획(duty, 전송값, 최대 시간) 출력 후 `yes` 입력해야 구동. 자동 정지 = odom 목표 도달 / 최대 시간 / 어느 1.5 s 구간이든 3 cm·5° 못 움직임 / 직진 중 30° 넘게 틀어짐 /
  LiDAR 장애물 (직진: 앞 0.25 m, 회전: 반경 0.22 m) / `/odom`·`/scan` 끊김 / Ctrl+C. 시작 전 LiDAR 로 앞쪽 공간·회전 반경 확인 후 부족하면 거부
- 검증: 합성 스캔 단위 시험 (이동 오차 4 mm, 회전 0.1°), fake STM32 + bringup (ROS_DOMAIN_ID=88 로 격리) 에서 직진·회전·정지·Ctrl+C·거부 확인.
  **LiDAR 경로는 실차에서 처음 실행** (fake 에는 LiDAR 없음)
- 순서: 직진 2~3회 → `wheel_radius` 적용 → bringup 재시작 → 회전 2~3회 → `wheel_separation` 적용 → 재시작 → 각 1회 확인
  (odom 회전각도 wheel_radius 에 비례하므로 직진 먼저)

**다음에 할 것**
1. 사용자 승인 후 실차 실행:
   - 직진: `ros2 run aircore_base floor_calib.py straight --distance 1.0 --speed 0.5` → duty 19 (전송 `M,-19,19`), 최대 8 s
   - 회전: `ros2 run aircore_base floor_calib.py rotate --angle 360 --turn 4.6` → duty 19 (전송 `M,19,19`, 좌회전), 최대 8 s
   - duty 19 에서 바닥 위 안 움직이면 (1.5 s 자동 정지) duty 22~25 로 올리는 것 다시 승인
   - **2026-11-24 17:09 첫 직진 (duty 19, `M,-19,19`): 1.6 s 만에 자동 정지, 사용자 "너무 느려서 거의 이동 못함"** → 바닥 dead band 가 duty 19 보다 큼
     → 다음: `--speed 0.6` (duty 23) → 안 되면 `--speed 0.7` (duty 26) → `--speed 0.8` (duty 30 = max_duty)
     (bringup 후 odom 이 뒤로 15 cm 인 건 사용자가 손으로 옮긴 것, 방향 문제 아님)
   - 17:2x 직진 duty 23: 1.5 s 동안 odom +1.9 cm, LiDAR +0.3 cm → 거의 정지 (바퀴 무부하 duty 20 은 0.37 m/s 였는데 바닥에선 23 도 부족).
     LiDAR 정지 상태 정합 품질: 일치 52%, rms 21 mm (기준 경계값 → 실제 이동 시 확인 필요)
   - 사용자: 전원·캐스터 문제 없음, **max_duty 60 으로 올리라고 지시** → `real_params.yaml` 30 → 60
   - floor_calib.py: `--duty N` 옵션 추가 (실행 중 max_wheel_speed 로 환산, max_duty 초과 시 거부), 속도 한도 1.7 m/s / 16 rad/s,
     직진 장애물 정지 거리 = 0.25 + 0.4 × 명령 속도 [m]. fake (도메인 88) 에서 --duty 35/40/−40, 거부 확인
   - 이어서 사용자 지시로 max_duty 60 → 100
   - 17:21~17:24 바닥 직진 (max_duty 100): duty 30 → 1.6 s 자동 정지, **duty 40 → 8 s 꽉 채워도 odom 1 m 미달 (평균 < 0.12 m/s)**,
     duty 50 ×2 → 약 1.5 s 에 정지 (이유 출력 없음), duty 30 → 정지. 사용자: "출력이 약하고 속도가 일정하지 않고 각도가 틀어짐"
   - 원인 분석 (11-24 17:30, 데이터로 확정한 것 / 가설 구분):
     · 확정: base_controller 는 **개루프** (양쪽에 같은 duty, 엔코더 피드백 없음) → 좌우 모터 차이가 그대로 방향 틀어짐.
       무부하 sweep 에서도 좌우 4~8% 차이, 같은 바퀴가 회차마다 17% 변함 (왼쪽 2.59 → 3.03)
     · 확정: 통신/CPU 문제 아님 (ENC 20 Hz, 최대 간격 65 ms, load 2.5)
     · 가설 A: 토크 부족 (모터 크기/정격 전압 미확인, 정격 250 rpm 은 사용자 확인).
       정지 직전 영역에서 돌기 때문에 마찰 차이가 큰 속도 차·끊김으로 나타남
     · 가설 B: 부하 시 전원 전압 강하 / 모터 드라이버 손실 (멀티미터로 주행 중 모터 전원 측정 필요)
     · 해결 방향: 바퀴별 속도 폐루프 (엔코더 PI) → 방향 틀어짐·속도 변동·dead band 해결. 단 모터 토크 자체가 모자라면 하드웨어 문제
   - **사용자 실험 (17:24): 2층의 배터리·젯슨을 잠시 내리고 duty 30 → "꽤 많이 이동"** (로그상 1.6 s 구동 = odom 1 m 도달로 추정, 약 0.6 m/s).
     → 무게(부하)에 비해 모터 토크 여유가 부족한 것이 주원인. duty 23 시험에서 odom 1.9 cm / LiDAR 0.3 cm → 바퀴가 미끄러진 게 아니라 거의 못 돈 것 (토크 부족)
     · URDF: 구동 바퀴는 차체 중앙 (x≈0), 볼 캐스터는 앞뒤 x=±0.08. CAD 질량 합 약 3.0 kg (배터리·젯슨 포함 여부 미확인)
     · ~~100% 환산 ~1000 rpm 추정~~ → **틀림. 사용자: 모터 JGA25-370, 정격 250 rpm (정격 전압 6V/12V 미확인).** 무부하 duty 30 에서 이미 257~275 rpm = 정격 무부하 속도 도달
       → duty-속도가 선형이 아님 (모터 정격보다 높은 전원 전압, 또는 드라이버 PWM 이 coast 방식이라 무부하에서 낮은 duty 로도 거의 최고속)
       → **무부하 sweep 으로 정한 max_wheel_speed 2.65 는 과대** (250 rpm × π × 0.065 m = 0.85 m/s). 짐 실은 바닥 주행 값으로 다시 정해야 함
     · 보정(2단계)은 최종 적재 상태(배터리·젯슨 장착)에서 해야 함
     · JGA25-370 250 rpm 급이면 (판매처마다 다르지만) 정지 토크가 수 kg·cm → 3~4 kg 로봇의 구름 저항(바퀴당 1 kg·cm 미만 추정)은 duty 40 이면 충분해야 함
       → 무게만으로는 설명 부족. 의심: 모터 드라이버 전류 한계 / 부하 시 전원 전압 강하 / 무게 실렸을 때 바퀴·축·캐스터 마찰
       → 필요한 정보: 모터 정격 전압 (라벨 6V/12V), 배터리 전압, 드라이버 칩, 주행 중 전압 (멀티미터)
   - floor_calib.py: 바퀴별 이동·평균 속도·속도 변동·정지 구간·좌우 비 출력 + `~/aircore_ws/calib_logs/*.json` 에 원자료 저장
   - **다음: bringup 재시작 (max_duty 100 반영) → `floor_calib.py straight --duty 30` → 35 → 40 → 45 → 50, 처음 제대로 움직이는 duty 에서 멈춤**
   - floor_calib.py 보강: 정지 시 움직인 양 표시, 명령과 반대로 3 cm/5° 움직이면 즉시 정지
   - ⚠ 나중 문제 (Nav2 전): 바닥 dead band 가 duty 19 이상이면 max_wheel_speed 2.65 기준 `/cmd_vel` 0.5 m/s 미만은 바닥에서 안 움직임.
     Nav2 보통 속도(0.2~0.3 m/s) 로는 못 감 → dead band 보상(최소 duty) 이나 속도 폐루프 필요. base_controller 변환 로직 수정이라 사용자 승인 필요

### 로그 분석 (2026-11-24 20:40, `calib_logs/` 15개 + `~/.ros/log` base_controller 로그)
사용자 보고: "왼쪽 모터 출력이 오른쪽보다 훨씬 높아서 자꾸 회전". 18:10~20:30 바닥 직진 duty 40 ×2, 50 ×8, 70 ×5.
- **어느 바퀴가 센지가 회차마다 바뀜** (같은 duty 50): 18:10 duty 40 왼쪽 0 / 오른쪽 27 cm, 19:50 오른쪽이 2배, 20:06 왼쪽 24 cm / 오른쪽 0,
  20:30 왼쪽 22 / 오른쪽 6 cm. 한쪽이 **완전히 멈추고** 다른 쪽만 돌아 제자리 선회 (직진 시험 중 yaw 71°, 96°, −84°, 87°, −197°)
  → 고정된 모터 차이만이 아니라, 토크 한계 근처에서 조금 더 버거운 쪽이 아예 서는 현상
- **명령은 그대로인데 두 바퀴가 같이 서서 몇 초간 정지** (base_controller 로그에 그동안 duty 50 계속 송신 확인):
  19:49:50 2 s 달린 뒤 5.5 s 정지, 19:50:09 1.2 s 뒤 4 s 정지 후 갑자기 오른쪽 1.0 m/s, 20:03:52 1 s 뒤 대부분 정지, 20:07:15 4 s 뒤 정지.
  평지·일정 duty 에서 두 바퀴가 동시에 서는 건 마찰 차이로 설명 안 됨 → **공통 원인 (모터 드라이버 과전류/과열 차단 또는 전원 강하) 의심**
- 20:29:14 duty 70: 바퀴 2.0~2.3 m/s (≈ 600~680 rpm) 로 부드럽게 가속 → 들어 올렸거나 가벼운 상태로 추정 (사용자 확인 필요).
  이때 **왼쪽이 약 10% 빠름**, 오른쪽은 순간 감속 (360 → 179 카운트/50 ms) 이 있음. (무부하 sweep duty 30 에선 오른쪽이 7% 빨랐음)
  · 바닥에서 odom 과 LiDAR 이동이 15% 안에서 일치 → ticks/반지름이 2배씩 틀린 건 아님 → **모터가 실제로 600 rpm 넘게 돎 → 정격 250 rpm 과 안 맞음**
    (250 rpm 형이 아니거나, 전원 전압이 모터 정격보다 훨씬 높음. 예: 6V 모터에 12~16V)
- **STM32 시리얼(FTDI) USB 끊김 6회**: 17:40, 19:45:51, 20:09:30, 20:09:34, 20:27:31, 20:27:50 (모두 주행 명령 없을 때, 1~14 s 뒤 재연결).
  20:27:51 재연결 직후 `ENC,0,0` → STM32 재시작 (카운트 리셋) → odometry 가 큰 점프 (enc_zeroed 오른쪽 +19578)
- **깨진 ENC 줄** (`EEEEEEENC,93,-137` 처럼 E 반복): 18:34~35, 19:47~48, 20:01~02, 주행 명령 없을 때 카운트가 변하는 중 (손으로 옮길 때로 추정)
- floor_calib.py 문제 발견·수정 (fake 도메인 88 검증): 정지 감지가 **처음 1.5 s 만** 봐서, 움직이다 서면 duty 50 으로 최대 8 s 까지 계속 밀었음
  → 어느 1.5 s 구간이든 3 cm/5° 못 움직이면 정지 ("움직이다 멈춤 … 드라이버/전원 차단 또는 걸림 의심"), **직진 중 30° 넘게 틀어지면 정지**,
  짧은 주행도 바퀴별 이동 거리 출력. (20:30:19 에 시작된 floor_calib 은 수정 전 코드, 줄자 입력 대기 중이었음)
- 결론: 좌우 차이는 **바퀴별 속도 폐루프(엔코더 PI)** 로 잡는 게 정석. 단 두 바퀴 동시 정지(공통 원인)는 폐루프로 해결 안 됨
  (PI 가 duty 를 더 올려 전류만 늘어남) → 드라이버/전원 먼저 확인. 필요한 정보: 모터 드라이버 칩, 배터리 전압, 모터 정격 전압, 주행 중 전압

## 이후 단계 메모
- **2단계 바닥 보정**: 위 `floor_calib.py` 절차. 원리: `wheel_radius` 새 값 = 기존 × 실제거리/odom거리, `wheel_separation` 새 값 = 기존 × odom회전각/실제회전각
- **3단계 SLAM**: `slam_toolbox`, `nav2_bringup` 설치돼 있음. TF 는 이미 REP-105 형태

## 알게 된 것 / 주의
- `src/` 의 config·scripts 는 `install/` 에 symlink 로 연결돼 있음 (symlink-install) → `real_params.yaml` 수정은 재빌드 없이 바로 반영
- 터미널에서 STM32 에 직접 보내는 `M,L,R` (stty/echo, picocom) 은 `invert_left/right` 를 거치지 않는다 → raw `M,20,20` 에서 왼쪽이 반대로 도는 건 정상. **raw 시험 결과로 invert 를 바꾸지 말 것.** 방향 판정은 `motor_test.py` (invert 적용 + 엔코더 기준)로
- 새 패키지를 빌드하면 이미 열어 둔 터미널은 `install/local_setup.bash` 를 다시 source 해야 인식된다 (`sllidar_ros2` not found 로 launch 가 죽은 적 있음 → 지금은 `bringup.launch.py` 가 노드 시작 전에 검사하고 중단)
- Humble 에서 `IncludeLaunchDescription` 의 launch_arguments 는 바깥 scope 로 샌다 → `bringup.launch.py` 는 scoped `GroupAction` 으로 감쌈
- RPLIDAR C1 은 180° 돌려서 장착돼 있음 → `aircore.urdf` 의 `laser_joint` yaw = π (실차 RViz 로 앞뒤·좌우 확인). 시뮬레이션도 같은 URDF 를 쓰므로 함께 바뀜 (시뮬에서 재확인은 안 함)
- Claude Code auto mode 에서 사용자 launch 프로세스 종료, 계획을 넘는 모터 구동은 권한 분류기에 막힌다 → 사용자가 직접 실행하거나 권한 설정 필요
- 바퀴 띄운 상태 속도는 무부하라 실제보다 빠르다

## 파일 맵 (2026-11-23 추가/수정, `~/aircore_ws/src/`)

| 파일 | 내용 |
|---|---|
| `aircore_base/config/real_params.yaml` | 실차 파라미터 (base_controller, odometry, sllidar_node) |
| `aircore_base/launch/bringup.launch.py` | 전체 bringup (인자: params_file, rviz, lidar, base, dry_run) |
| `aircore_base/launch/odom_view.launch.py` | base + odom + odom 전용 RViz, 미측정 키 경고 |
| `aircore_base/rviz/bringup.rviz`, `odom.rviz` | RViz 설정 (Top-down, 화면 위 = 전진 +x) |
| `aircore_base/scripts/measure_ticks.py` | 바퀴 손으로 N바퀴 → ticks_per_rev 측정 (ROS 토픽만 사용) |
| `aircore_base/scripts/apply_measurements.py` | r, L, ticks 를 real_params.yaml 두 섹션에 일관되게 기록 (범위 검사) |
| `aircore_base/scripts/motor_test.py` | 제한된 모터 테스트 (duty ≤ max_duty, ≤ 3 s, ENC 끊기면 정지, 포트 점유 시 거부) |
| `aircore_base/scripts/floor_calib.py` | (11-24) 2단계 바닥 보정: 제한된 직진/회전 + LiDAR ICP 측정 → r, L 제안 |
| `aircore_base/setup.py`, `package.xml` | rviz/scripts 설치, exec_depend 추가 |
| `aircore_description/urdf/aircore.urdf` | `laser_joint` yaw = π |
| `sllidar_ros2/` | Slamtec 공식 드라이버 (커밋 3430009, udev/sudo 스크립트는 실행 안 함) |

테스트 도구 (`~/aircore_ws/tools/`, 패키지에 설치 안 됨):
- `fake_stm32.py`: pty 가짜 STM32 (하드웨어 없이 `motor_test.py` 시험, `--params` 로 포트를 바꾼 yaml 사용)
- `scan_probe.py`: `/scan` 근거리 물체를 base_footprint 기준 앞/뒤/좌/우로 요약
