# Repository Guidelines

## 프로젝트 구조

이 프로젝트는 STM32F407VGT6에서 Jetson UART 명령으로 좌우 모터를 제어하고 엔코더 값을 전송하는 STM32CubeIDE 펌웨어다. `Core/Src/main.c`에 명령 처리와 제어 루프가 있으며, `Core/Inc/`에는 헤더가 있다. GPIO 대체 기능과 클록 설정은 `stm32f4xx_hal_msp.c`, 인터럽트 진입점은 `stm32f4xx_it.c`에서 확인한다.

`uart_test.ioc`는 CubeMX 설정 원본이다. `Drivers/`에는 CMSIS와 HAL, `Middlewares/`와 `USB_HOST/`에는 기존 USB 지원 코드가 있다. `Debug/`는 생성된 빌드 결과이며 직접 수정하지 않는다. `validation/`은 검사 스크립트와 결과, `tools/`는 Jetson 시험 도구를 보관한다. 배선과 실물 시험 절차는 `FIRMWARE_GUIDE.md`를 참고한다.

## 빌드 및 개발 명령

프로젝트 루트에서 다음 명령을 실행한다.

- `powershell -ExecutionPolicy Bypass -File validation/build.ps1`: 설치된 STM32 도구로 Debug 전체를 다시 빌드한다.
- `node validation/check-config.js`: IOC 핀 배치, PWM 설정, 엔코더 모드와 인터럽트 연결을 검사한다.
- `node validation/generate-tests.js`: 현재 펌웨어에서 호스트 시험 코드를 추출한다.
- `cmd /c validation\run-host-tests.cmd`: MSVC로 모의 HAL 시험을 빌드하고 실행한다.

스크립트의 도구 설치 경로가 현재 환경과 맞는지 확인한다. IDE에서는 Debug 구성을 선택하고 Build Project를 실행한다. 다운로드와 디버깅에는 기존 `uart_test Debug.launch`를 사용한다.

## 코드 스타일과 설정 관리

C 코드는 주변 코드에 맞춰 공백 두 칸을 사용하고 중괄호는 다음 줄에 배치한다. 내부 함수는 `static`으로 제한하며 `parseCommand`처럼 camelCase 이름을 따른다. 상수는 `MOTOR_TIMEOUT_MS`처럼 대문자와 밑줄을 사용한다. 고정 폭 정수형과 크기가 제한된 버퍼를 우선 사용한다.

별도 포매터나 린터 설정은 없다. 변경 범위를 필요한 부분으로 제한하고 기존 USER CODE 마커를 보존한다. CubeMX 재생성 전에는 백업하며, 재생성 후 IOC와 실제 초기화 코드를 비교한다. HAL과 미들웨어의 직접 수정은 피한다.

## 검증 기준

자동 검사는 명령 파싱, 독립 PWM, 방향 전환, 통신 오류, 타임아웃과 엔코더 송신을 다룬다. 별도의 커버리지 비율 기준은 없다. 제어 로직 변경 후에는 시험 코드를 다시 추출하고 전체 빌드의 오류와 경고를 확인한다. 호스트 시험 통과를 실제 모터 시험 완료로 보고하지 않는다.

## 커밋과 Pull Request

현재 작업 사본에는 Git 이력이 없어 기존 커밋 관례를 확인할 수 없다. 새 커밋에는 `fix: stop motors on UART overflow`처럼 변경 목적이 드러나는 제목을 권장한다. PR에는 변경 이유, 영향받는 핀과 주변장치, 빌드 및 시험 결과, 미검증 하드웨어 항목을 적는다. 관련 이슈가 있다면 연결하고, 타이밍 변경에는 측정 결과를 첨부한다. 생성된 바이너리와 백업은 소스 변경과 구분한다.
