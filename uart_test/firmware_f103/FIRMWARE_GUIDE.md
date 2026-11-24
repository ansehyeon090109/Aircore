# Aircore NUCLEO-F103RB 펌웨어

대상은 MB1136 NUCLEO-F103RB의 STM32F103RBT6입니다. STM32CubeIDE에서 기존 프로젝트 가져오기로 이 폴더를 선택합니다. 프로젝트 이름은 `firmware`입니다.

## Morpho 결선

| 신호 | 기존 F407 | NUCLEO-F103RB | 설정 |
|---|---|---|---|
| L298N ENA | PB4 | PA6 / CN10-13 | TIM3_CH1 PWM |
| L298N ENB | PB5 | PA7 / CN10-15 | TIM3_CH2 PWM |
| L298N IN1 | PB0 | PB0 / CN7-34 | GPIO |
| L298N IN2 | PB1 | PB1 / CN10-24 | GPIO |
| L298N IN3 | PE7 | PB12 / CN10-16 | GPIO |
| L298N IN4 | PE8 | PB13 / CN10-30 | GPIO |
| 왼쪽 엔코더 A/B | PA0 / PA1 | PA0 / CN7-28, PA1 / CN7-30 | TIM2_CH1/2 |
| 오른쪽 엔코더 A/B | PA15 / PB3 | PB6 / CN10-17, PB7 / CN7-21 | TIM4_CH1/2 |
| USB-TTL RX ← STM32 TX | PD8 | PB10 / CN10-25 | USART3_TX |
| USB-TTL TX → STM32 RX | PD9 | PB11 / CN10-18 | USART3_RX |
| 공통 GND | GND | CN7-19/20 또는 CN10-20 | L298N, 양쪽 엔코더, USB-TTL |
| SWDIO / SWCLK | PA13 / PA14 | PA13 / CN7-13, PA14 / CN7-15 | 보드 내부 ST-LINK 연결 |
| ST-LINK VCP | — | PA2/PA3, 보드 내부 연결 | USART2, 로봇 프로토콜 미사용 |

내장 ST-LINK는 CN2 점퍼 두 개를 장착하고 ST-LINK USB를 PC에 연결합니다. Jetson은 별도의 3.3V USB-TTL 어댑터로 USART3에 연결합니다. USB-TTL의 전원 핀은 연결하지 않습니다.

L298N ENA/ENB 점퍼를 제거하고 좌우 모터는 각각 OUT1/OUT2와 OUT3/OUT4에 연결합니다. 모터와 엔코더 전원은 각 장치 정격에 맞게 공급합니다. 엔코더 A/B는 3.3V 로직으로 입력합니다. 오픈컬렉터 출력이면 두 신호를 3.3V에 외부 풀업합니다. 모터 리셋 구간에도 PWM 입력을 Low로 유지하려면 ENA/ENB에 외부 풀다운을 사용합니다.

[NUCLEO-F103RB UM1724 표 27 및 §7.4, §7.9–7.10](https://www.st.com/resource/en/user_manual/um1724-stm32-nucleo64-boards-mb1136-stmicroelectronics.pdf) · [STM32F103RB 데이터시트](https://www.st.com/resource/en/datasheet/stm32f103rb.pdf) · [RM0008 AFIO 및 TIM2–TIM4](https://www.st.com/resource/en/reference_manual/rm0008-stm32f103xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)

## 동작

- 보드 리비전별 HSE 배선 차이와 관계없이 내부 HSI 8MHz를 사용합니다. HSI/2 × 16으로 SYSCLK 64MHz, APB1 32MHz, APB1 타이머 64MHz입니다. PWM은 64MHz ÷ 64 ÷ 1000 = 1kHz입니다.
- 일반 주변장치 AFIO 리맵을 사용하지 않습니다. JTAG를 해제하고 SWD는 유지합니다. USART3은 인터럽트로 송수신하며 DMA 채널을 사용하지 않습니다.
- Jetson에서 USART3으로 115200 8N1로 `M,left,right`를 보내고 CR, LF 또는 CRLF로 끝냅니다. 듀티 범위는 -100~100, 마지막 정상 명령 이후 정지 시간은 300ms입니다. 방향 전환 시 PWM 0 구간을 약 2ms 유지합니다.
- TIM2와 TIM4는 16비트 TI12 모드입니다. 메인 루프에서 두 카운터를 읽고, 16비트 모듈러 차이를 부호 있는 변화량으로 해석한 뒤 32비트 누적값에 더합니다. 이 방식은 연속 샘플 사이 실제 이동이 32768 카운트 미만일 때 양방향 랩어라운드를 복원합니다. 디버거 정지 상태에서도 엔코더가 움직이면 이 조건이 깨질 수 있습니다.
- 약 50ms마다 `ENC,left,right\r\n`을 전송합니다. 두 값은 32비트 부호 없는 십진수입니다. 선택적인 `PIN` 진단 출력은 기본적으로 꺼져 있습니다. F407 대상 USB Host·I2S는 모터, 엔코더, Jetson 경로에 사용되지 않아 이 프로젝트에 포함하지 않았습니다.

## 빌드와 검증

```powershell
powershell -ExecutionPolicy Bypass -File validation/build.ps1
node validation/check-config.js
node validation/generate-tests.js
cmd /c validation\run-host-tests.cmd
```

ARM GCC 결과는 `Debug/firmware.elf`와 `Debug/firmware.hex`입니다. CubeIDE에서 `firmware Debug.launch`로 내장 ST-LINK/SWD를 사용합니다.

CubeMX 6.15 명령줄에서 IOC를 불러왔으나 격리 환경의 생성 결과에는 주변장치 초기화 함수 본문이 빠졌습니다. 이 프로젝트의 초기화 코드는 F1 HAL로 직접 구성하고 IOC와 대조해 빌드했습니다. CubeIDE에서 코드를 재생성할 때는 먼저 백업하고 초기화 함수 및 USER CODE 블록을 비교합니다.

## 보드 시험

1. 모터 전원을 차단하고 보드를 올립니다. ENA/ENB 및 IN1–IN4가 Low인지 확인합니다.
2. USART3으로 `ENC`가 약 20Hz로 수신되는지 확인합니다. 좌우 엔코더를 번갈아 정·역회전하여 해당 필드만 변하는지 보고 16비트 경계를 넘어도 누적값이 이어지는지 확인합니다.
3. 바퀴를 지면에서 띄우고 모터 전원을 인가합니다. `M,30,0`, `M,0,30`, `M,50,25` 순으로 채널별 동작과 1kHz PWM, 듀티를 측정합니다.
4. 정방향에서 역방향으로 명령을 바꾸고 PWM이 먼저 Low가 되며 약 2ms 뒤 방향이 바뀌는지 측정합니다.
5. CR/LF/CRLF, 잘못된 명령, 긴 수신 라인, UART 장애, 송신 중단을 시험합니다. 오류 후 양쪽 정지, 정상 명령 복구 및 300ms 타임아웃을 확인합니다.
