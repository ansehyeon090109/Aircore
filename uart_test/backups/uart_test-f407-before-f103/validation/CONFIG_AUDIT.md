# IOC 및 프로젝트 설정 검토 결과

## 수정 사항

- 재검사 시작 시 IOC에 I2C1과 PB8/PB9 할당이 남아 있었으며 코드 생성 목록에 TIM3가 없었다. 당시 파일은 `backups/config_audit_20261117_175200/`에 보관했다.
- I2C1 할당과 초기화 목록을 제거하고 TIM3 초기화 목록, PWM 모드·극성·초기 Pulse, UART 115200 8N1을 명시했다. 기존 USB_HOST 목록은 유지했다.
- TIM3 MSP 함수들을 USER CODE 1 밖으로 옮겼다. CubeMX가 생성하는 함수를 사용자 코드로 보존해 재생성 시 중복 정의하는 문제를 방지한다. 실제 재생성 시험은 아래 제한 때문에 완료하지 못했다.

## 통과한 검사

| 항목 | 결과 |
|---|---|
| IOC 핀 중복 및 주변장치 등록 | 통과 |
| 설치된 STM32F407 MCU 데이터베이스의 PWM·엔코더·UART 핀 기능 | 통과 |
| TIM3 Internal Clock, PSC 83, ARR 999, CH1/CH2 PWM1 High, 초기 0 | 코드와 IOC 일치 |
| TIM2/TIM5 TI12, 32비트 ARR, 입력 필터 4 | 코드와 IOC 일치 |
| HSE 8MHz, SYSCLK 168MHz, APB1 타이머 84MHz | 코드·HAL 설정·IOC 일치 |
| USART3 IRQ, 시작 코드의 IRQ 벡터, 한 바이트 인터럽트 수신 | 통과 |
| Debug·Release 대상 MCU와 소스 루트, Flash 링커 스크립트 | 통과; 실제 빌드는 Debug |
| Flash 1024KB, RAM 128KB, CCMRAM 64KB | 대상 MCU 설정과 일치 |
| 디버그 ELF 경로, SWD 활성, SWV 비활성 | 통과; PB3의 SWO 사용 방지 |
| 최신 소스 전체 Debug 강제 재빌드 | 성공, 컴파일 오류 0 / 경고 0 |
| 파서·모터·타임아웃·UART 오류·엔코더 모의 HAL 테스트 | 모두 통과 |

검사 명령은 `node validation/check-config.js`, `powershell -ExecutionPolicy Bypass -File validation/check-project.ps1`다. 빌드와 호스트 시험 방법은 `FIRMWARE_GUIDE.md`를 참고한다. 결과는 `config-check.log`, `project-check.log`, `build-debug.log`, `host-tests.log`에 기록했다.

## CubeMX 자체 검증의 제한

[ST 공식 CLI 절차](https://dev.st.com/stm32cube-docs/stm32cubemx/6.18.0/en/docs/markup/CubeMX_CLI.html)를 참고해 설치된 CubeMX 6.15 JAR로 `config load`, `config saveext`, `csv pinout`을 시도했다. 별도 실행 환경에서 레지스트리 접근 및 외부 패키지 데이터베이스 오류가 발생했고, `VP_RIF_VS_RIF1`을 찾지 못한다는 메시지 후 읽기·저장이 완료되지 않았다. 실제 프로젝트 IOC에는 이 RIF 항목이 없다. 원인까지 확정하지 못했으므로 **CubeMX 자체 검증 성공 또는 재생성 성공으로 처리하지 않는다**. 로그는 `cubemx-audit.log`다.

기존 IOC 편집기는 저장하지 않고 닫아 이전 메모리 상태가 수정 파일을 덮어쓰지 않도록 했다. 현재 파일을 IDE에서 다시 열어 확인할 수 있다.

## 실물 확인 항목

실제 모터 회전, PWM 파형, 엔코더 신호와 타임아웃 정지 시간은 보드에서 확인해야 한다. 프로젝트가 지정한 Discovery 보드를 사용한다면 PA0의 사용자 버튼 등 보드 회로 연결도 확인한다. 이 검토는 Flash 다운로드나 모터 구동을 수행하지 않았다.
