# aircore_description — 자동 생성 URDF 패키지

Fusion 360 없이, 업로드하신 개별 STL(`aircore.stl` 전체 조립본 + 14개 파트)의 **월드 좌표**를 직접
분석해서 조인트 위치를 역산하고 URDF를 생성했습니다. 아래 가정들은 실제 CAD 조인트 정보가 없어서
추정한 부분이니 **반드시 검토 후 필요하면 수정**해 주세요.

## 확인된 사실 (좌표 분석으로 검증됨)
- 모든 개별 STL이 `aircore.stl`과 동일한 월드 좌표계를 공유함 → 실제 배치 좌표를 그대로 사용 가능
- `support_1`/`support_2`는 STL이 1개씩만 제공되었고, (-80, 80) 코너에 위치 → 나머지 3개 코너
  (80,80), (-80,-80), (80,-80)는 **좌우/전후 대칭 미러링으로 생성**했습니다.
  실제 부품이 완전히 대칭이 아니라면 이 부분은 틀릴 수 있습니다.
- base_link(1층) z: 0-41mm, floor_2(2층) z: 56-84mm, floor_3(3층) z: 118-146mm
- support_1: z 8-68mm (1층-2층 연결), support_2: z 71-131mm (2층-3층 연결)
- left/right_wheel: Y축 기준 원형 단면 확인 → 회전축 = Y축, `continuous` 조인트
- left/right_support_wheel(캐스터): x=∓63, y=0 위치, 지름 25mm의 작은 볼

## 임의로 결정한 것 (검토 필요)

### 1. 캐스터(support_wheel)를 "fixed"로 처리
URDF에는 진짜 3D 자유회전(볼) 조인트가 없습니다. 요청하신 "360도 자유회전"에 가장 가까운 표현은:
- Gazebo에서 **fixed 조인트 + 낮은 마찰(mu1/mu2=0.01)** 로 처리 (물리 엔진이 미끄러지듯 굴러가게 함)
현재 이 방식으로 구현했습니다. 정확한 전방향 이동이 꼭 필요하면 캐스터 2-link 모델
(수직 회전축 + 수평 회전축)이나 Gazebo의 `libgazebo_ros_planar_move` 같은 플러그인 접근이 더 정확합니다.

### 2. 질량/재질 밀도 가정 — 실제 값으로 교체 권장
| 파트 | 방식 | 가정값 |
|---|---|---|
| 구조부 (base_link, floor_2/3, support_1/2, wheel, support_wheel) | STL 부피 × PLA 밀도(1.24 g/cm³) | 계산값 (표 참고) |
| battery | 지오메트리 무시, 일반 LiPo 추정 | 300 g |
| driver (모터 드라이버) | 일반 소형 보드 추정 | 50 g |
| stmf407 (STM32F407 보드) | 공식 스펙 유사값 | 30 g |
| JetsonNano | 방열판/케이스 포함 추정 | 240 g |
| LiDAR | **SLAMTEC 공식 비교표 기준 (사용자 제공, 확정값)** | 110 g |

전자부품은 실제 데이터시트 무게로 꼭 바꿔주세요. 구조부가 실제로 PLA가 아니면(예: 알루미늄,
아크릴) `build_urdf.py`의 `PLA = 1.24e-6` 값을 재질 밀도(kg/mm³)로 바꿔서 재생성해야 합니다.

### 3. 조인트 원점(anchor) 선택
실제 Fusion 조인트 데이터가 없어서, 각 부품의 **STL 자체 무게중심**을 조인트 기준점으로 사용했습니다.
- `fixed` 조인트는 기준점이 어디든 결과가 동일하므로 문제 없음
- `continuous`(바퀴) 조인트는 회전축이 실제 원형 단면 중심과 일치하는 것을 좌표로 확인했으므로 안전함

### 4. 트리 구조를 단순화(flat tree)
support_1/2, floor_2/3 등이 실제로는 서로를 거쳐 연결되는 직렬 구조일 수 있지만(예: base_link →
support_1 → floor_2 → support_2 → floor_3), 전부 `fixed`라서 기구학적으로는 결과가 같습니다.
그래서 전부 `base_link`에 바로 붙이는 flat tree로 단순화했습니다.

## 파일 구조
```
aircore_description/
├── CMakeLists.txt, package.xml
├── meshes/            (20개 STL: 원본 14개 + support 미러링 6개)
├── urdf/
│   ├── aircore.xacro      (링크+조인트 본체)
│   ├── materials.xacro
│   ├── aircore.trans      (바퀴 2개 transmission)
│   └── aircore.gazebo     (마찰/색상 + LiDAR ray 센서 플러그인)
└── launch/
    ├── display.launch (rviz)
    ├── gazebo.launch
    ├── controller.launch / controller.yaml (바퀴 velocity controller)
```

## LiDAR 기능
`lidar` 링크에 Gazebo classic(ROS1) `libgazebo_ros_laser.so` ray 센서 플러그인을 넣었습니다.

**RPLiDAR C1 확정 스펙** (SLAMTEC 공식 비교표, 사용자 제공 이미지 기준):
| 항목 | 값 |
|---|---|
| 거리범위 (흰색 물체) | 0.05 ~ 12 m |
| 거리범위 (검은색 물체) | 0.05 ~ 6 m |
| 스캔레이트 | 8~12 Hz (URDF엔 중간값 10Hz 적용) |
| 샘플레이트 | 5000 Hz |
| 각분해능 | 0.72° (→ 회전당 500 샘플) |
| 측정각 | 360° |
| 크기 | 55.6 × 55.6 × 41.3 mm (STL 실측과 정확히 일치 — 모델 확인됨) |
| 무게 | 110 g |
| 사용환경 | 실내·실외(IP54) |

360도, 10Hz, 최대 12m로 `/scan` 토픽에 publish합니다.
**주의**: Gazebo의 기본 ray 센서는 물체 반사율(흰색/검은색)을 구분하지 않으므로, 시뮬레이션에서는
항상 12m 범위로 동작합니다. 실제 하드웨어는 어두운/비반사 물체에서 6m로 감소하니, 실환경 테스트
시 이 차이를 감안하세요. ROS2/최신 Gazebo(Ignition/Harmonic)를 쓰신다면 플러그인 이름이 다르므로
알려주시면 맞춰드리겠습니다.
