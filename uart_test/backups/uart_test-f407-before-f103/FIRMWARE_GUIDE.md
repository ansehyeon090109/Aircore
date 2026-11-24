# Aircore STM32F407 펌웨어 검증 안내

추가 IOC 및 프로젝트 설정 점검 결과와 CubeMX 자체 검증의 제한은 `validation/CONFIG_AUDIT.md`에 정리했다.

## 적용 범위와 결과

대상은 STM32F407VGT6, STM32CubeIDE 1.19.0의 `uart_test` 프로젝트다. 요청한 `uart_test(1).ioc`는 현재 폴더에 없어 `uart_test.ioc`를 수정했다. 원본 전체는 `backups/20261117_174247/`에 보관했다. 반영 전 비교는 `validation/*-before-apply.diff`, 최종 비교는 `validation/*-final.diff`에 있다.

GNU Tools for STM32 13.3.rel1로 Debug 전체를 강제 재빌드했다. 결과는 `validation/build-debug.log`에 기록된다. `Debug/uart_test.elf`와 `Debug/uart_test.hex`가 생성된다. 호스트 테스트는 실제 `main.c`에서 파서, 모터 제어, 메인 루프 본문, UART 콜백을 추출하고 HAL을 모의해 실행한다. 물리적인 PWM 파형, 배선, 모터 회전과 엔코더 펄스는 이 테스트의 검증 범위가 아니다.

## 핀 설정

| 연결 | MCU 핀 | 설정 |
|---|---|---|
| L298N ENA, 왼쪽 | PB4 | AF2, TIM3_CH1 PWM |
| L298N ENB, 오른쪽 | PB5 | AF2, TIM3_CH2 PWM |
| L298N IN1 / IN2 | PB0 / PB1 | Push-pull 출력, 초기 Low |
| L298N IN3 / IN4 | PE7 / PE8 | Push-pull 출력, 초기 Low |
| 왼쪽 엔코더 A / B | PA0 / PA1 | AF2, TIM5_CH1 / CH2, TI12 |
| 오른쪽 엔코더 A / B | PA15 / PB3 | AF1, TIM2_CH1 / CH2, TI12 |
| USB-TTL RX / TX | PD8 / PD9 | AF7, USART3 TX / RX, 115200 8N1 |
| 디버거 SWDIO / SWCLK | PA13 / PA14 | Serial Wire, JTAG/SWO 사용 안 함 |
| 사용하지 않는 I2C | PB8 / PB9 | IOC 할당 해제, I2C1 클록/핀 초기화 제거 |

동일 핀 중복 할당은 없다. 기존 I2S3, USB 및 보드 지원 파일은 유지한다. 실제 보드가 프로젝트에 지정된 Discovery라면 PA0에 연결된 사용자 버튼 등 보드 부품의 영향도 확인한다. MCU 핀 할당의 충돌 검사와 보드 회로의 전기적 충돌 검사는 별개다.

TIM3는 Internal Clock, PSC=83, ARR=999, Up, DIV1, PWM1, High, 두 채널 Pulse=0이다. 기존 HSE 8MHz, SYSCLK 168MHz, APB1 타이머 클록 84MHz를 유지하므로 PWM은 1kHz다. TIM2/TIM5는 PSC=0, ARR=4294967295, TI12, 입력 필터 4를 유지한다. 핀 대체 기능과 타이머 폭은 [ST 데이터시트](https://www.st.com/resource/en/datasheet/stm32f407ve.pdf)를 기준으로 검토했다.

## 빌드와 자동 검사

프로젝트 루트에서 실행한다.

```powershell
powershell -ExecutionPolicy Bypass -File validation/build.ps1
node validation/check-config.js
node validation/generate-tests.js
cmd /c validation\run-host-tests.cmd
```

빌드 스크립트는 `C:/ST/STM32CubeIDE_1.19.0/` 설치 경로를 사용한다. 호스트 테스트는 설치된 MSVC 14.50의 x64 컴파일러와 Windows CRT를 사용한다. 다른 환경에서는 스크립트의 도구 경로를 조정한다. IDE에서는 프로젝트를 새로 고침한 뒤 Debug 구성을 선택하고 **Project → Build Project**로 빌드하며, `uart_test Debug.launch`로 ST-LINK 디버깅을 시작한다.

IOC 키와 PWM 모드는 설치된 CubeMX 6.15 데이터베이스와 대조했다. 이 작업에서 CubeMX GUI를 열어 저장하거나 코드를 재생성하지는 않았다. 나중에 재생성할 때는 먼저 백업하고 USER CODE 외 초기화 코드도 반드시 비교한다.

## 명령과 정지 동작

명령은 ASCII `M,left,right`이며 각 값은 -100~100이다. LF, CR, CRLF를 지원한다. 아래 명령에는 반드시 개행을 붙인다.

| 명령 | 기대 동작 |
|---|---|
| `M,0,0` | 양쪽 정지 |
| `M,30,0` | 왼쪽만 정방향 30% |
| `M,0,30` | 오른쪽만 정방향 30% |
| `M,50,25` | 좌우 서로 다른 듀티 |
| `M,-30,-30` | 양쪽 역방향 |
| `M,30,-30` | 좌우 반대 방향 |
| `M,101,0` 또는 `BAD` | 양쪽 정지 |

정상 명령은 50~100ms 간격으로 반복한다. 마지막 정상 명령 처리 후 300ms가 지나면 정지한다. 잘못된 형식, 빈 명령, NUL, 23자를 넘는 수신 라인, UART 오류와 처리 대기 슬롯 초과도 정지한다. 지나치게 빠른 연속 명령을 보내면 안전 정지가 발생할 수 있다. 오류 라인은 다음 개행까지 버리고 이후 정상 명령으로 복구한다.

부팅 시 방향 출력과 두 PWM은 0이다. 회전 방향이 반전되면 해당 채널 PWM을 먼저 0으로 만들고 2ms 대기한 뒤 방향을 바꾼다. 이 대기는 메인 루프에서만 수행하며 수신·송신 인터럽트는 계속 동작한다. 정상 루프에서 오류 정지 응답에는 최대 두 채널의 방향 전환 대기 약 4ms와 처리 시간이 추가될 수 있다. `Error_Handler`는 두 PWM 채널을 비활성화한다.

## Jetson 테스트 및 엔코더 해석

Jetson에서 pyserial이 설치되어 있어야 한다. 포트 이름은 실제 USB-TTL 장치에 맞춘다.

```bash
python3 tools/jetson_test.py --port /dev/ttyUSB0
python3 tools/jetson_test.py --left 30 --right 0 --duration 2
python3 tools/jetson_test.py --left 0 --right 30 --duration 2
python3 tools/jetson_test.py --left 30 --right -30 --duration 2
python3 tools/jetson_test.py --left 30 --right 30 --duration 2 --timeout-test
```

첫 명령은 정지 상태로 수신만 확인한다. 스크립트는 구동 명령을 50ms마다 보내며 종료 시 `M,0,0`을 전송한다. `--timeout-test`는 지정 시간 뒤 600ms 동안 명령을 보내지 않고 수신을 계속하므로 실제 300ms 자동 정지를 관찰할 수 있다. 이 Python 도구는 Jetson에서 실행할 보조 도구이며 현재 Windows 환경에서 실제 시리얼 실행은 하지 않았다.

STM32는 약 50ms마다 `ENC,left,right\r\n`을 송신한다. 왼쪽은 TIM5, 오른쪽은 TIM2의 **부호 없는 32비트 원시 누적 카운터**다. 기존 호환성을 위해 음수 출력이나 구간 펄스 수로 변경하지 않았다. 역회전 시 0에서 4294967295로 이동하는 것은 정상이다. Jetson의 구간 변화량은 다음과 같이 계산한다.

```python
delta = ((current - previous + 0x80000000) & 0xFFFFFFFF) - 0x80000000
```

이 계산은 구간 이동량이 2^31 펄스 미만일 때 양방향 래핑을 처리한다. 전원 재시작 후에는 이전 기준값을 새로 잡는다. 엔코더 A/B 순서와 모터 극성에 따른 정방향 카운트 부호는 실제 바퀴 회전으로 확인한다. 송신 중에는 고정 버퍼를 유지하고 다음 송신을 건너뛰므로 버퍼를 덮어쓰지 않는다.

## 실제 하드웨어 시험 순서

1. 바퀴를 지면에서 띄우고 모터 전원을 끈다. L298N의 ENA/ENB 점퍼를 제거하고 PWM 입력을 연결한다. USB-TTL은 3.3V 로직을 사용하며 GND를 공통 연결한다.
2. MCU에 펌웨어를 다운로드한다. 모터 전원을 켜기 전 PB0/PB1/PE7/PE8 Low, PB4/PB5 듀티 0을 측정한다. 리셋 중에도 정지가 필요하면 EN 입력에 외부 풀다운을 사용한다.
3. UART에서 `ENC`가 약 20Hz로 수신되는지 확인한다. 왼쪽 바퀴만 손으로 돌려 left가, 오른쪽만 돌려 right가 변하는지 확인하고 역회전도 검사한다.
4. 모터 전원을 켜고 `M,30,0`, `M,0,30`으로 각각 독립 구동을 확인한다. PB4/PB5에서 1kHz 및 해당 듀티를 측정한다.
5. 양쪽을 서로 다른 듀티로 구동하고 반전 명령을 보낸다. 로직 분석기로 해당 PWM의 0 구간이 방향 변경보다 앞서는지 확인한다.
6. `M,101,0`, `BAD`, 긴 라인, CR/LF/CRLF를 시험한다. 오류 시 양쪽 정지와 이후 정상 명령 복구를 확인한다.
7. `--timeout-test`로 마지막 명령 약 300ms 뒤 정지하는지 측정한다. 전송 프로그램 종료·케이블 분리도 검사한다.
8. 다시 부팅해 무명령 상태 정지를 확인한 뒤 낮은 듀티부터 지상 시험을 진행한다. 실측 결과는 자동 검사 결과와 별도로 기록한다.
