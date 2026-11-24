# aircore_ws (스마트팜 자율주행 차량, ROS 2 Humble, Jetson Orin Nano)

- 작업을 시작하면 **`STATUS.md` 를 먼저 읽는다** (현재 단계, 미해결 문제, 다음 할 일, 재개 명령). 배경·시뮬레이션은 `README.md`.
- 작업이 한 단계 끝나거나 세션을 마칠 때 `STATUS.md` 를 갱신한다.
- 사용자와는 한국어로 대화한다.

## 실차 안전 규칙 (사용자가 매번 지시한 것)
- 구동 모터를 움직이는 동작은 **단계마다 duty·시간을 먼저 제시하고 승인받은 뒤**에만. 승인 없이 `/cmd_vel` publish, `M,0,0` 이외 명령 송신 금지.
- `max_wheel_speed` 를 0 이 아닌 값으로 바꾸는 것도 승인 필요 (그 순간 `/cmd_vel` 로 주행 가능해짐).
- sudo 실행 금지. 설치가 필요하면 명령만 알려준다.
- 포트는 by-id 경로만: STM32 = FTDI FT232R, LiDAR = CP2102N. STM32 작업에 LiDAR 포트를 열지 않는다.
- 모든 ROS 명령은 `ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1`.
- 확인 안 된 값을 가정하지 않는다. 테스트용 임시 값은 scratchpad 에만.
- `config/base_params.yaml` 원본, `base_controller.py` 통신/안전 로직, `odometry.py` 계산 로직은 수정하지 않는다.
- base_controller 가 실행 중이면 STM32 포트를 다른 프로그램이 열면 안 된다 (`motor_test.py` 는 자체 검사함).
