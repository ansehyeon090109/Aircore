# RPlidar C1 URDF 작성 가이드

## 1. 센서 사양

### 물리적 특성
| 항목 | 값 |
|------|-----|
| 크기 | 55.6 × 55.6 × 41.3 mm |
| 무게 | 110g (0.11 kg) |
| 형태 | 원형 (원통형) |

### 성능 사양
| 항목 | 값 |
|------|-----|
| 측정 범위 | 0.05~12m (흰색 물체 70% 반사율) |
| 스캔 각도 | 360° |
| 스캔 주파수 | 10Hz (전형값) |
| 샘플링 주파수 | 5kHz |
| 각도 해상도 | 0.72° |
| 정확도 | ±30mm |
| 최소 블라인드 존 | 0.05m |

### 전기 사양
| 항목 | 값 |
|------|-----|
| 전원 | 5V DC |
| 전류 | 230mA |
| 전력 | 1.15W |
| 통신 | UART (TTL) / USB |

---

## 2. URDF 파일 구조

### 기본 링크 구성

```
rplidar_c1_frame (마운팅 프레임)
    ↓ (fixed joint)
rplidar_c1_base (센서 본체)
    ↓ (fixed joint)
rplidar_c1_laser (레이저 중심점)
    ↓ (continuous joint - 회전축)
rplidar_c1_rotating (회전 프레임)
```

### 각 링크의 역할

1. **rplidar_c1_frame**: 로봇에 부착하기 위한 마운팅 베이스
2. **rplidar_c1_base**: 센서의 실제 본체 (원통형)
3. **rplidar_c1_laser**: 레이저 스캔이 발생하는 원점 (센서 중심)
4. **rplidar_c1_rotating**: 회전 축 (시뮬레이션용)

---

## 3. 기하학적 치수 계산

### 센서 본체 모델링
- **형태**: 실린더 (원통형)
- **반지름**: 55.6mm / 2 = 27.8mm = 0.0278m
- **높이**: 41.3mm = 0.0413m

### 질량 및 관성 모멘트
```
질량 (m) = 0.11 kg

관성 모멘트 (원통형 실린더):
- Ixx = Iyy = (1/12) × m × (3r² + h²)
       = (1/12) × 0.11 × (3×0.0278² + 0.0413²)
       ≈ 0.0001 kg⋅m²

- Izz = (1/2) × m × r²
      = 0.5 × 0.11 × 0.0278²
      ≈ 0.0000425 kg⋅m²
```

---

## 4. ROS에서 사용하기

### 4.1 Gazebo 시뮬레이션

**gazebo_rplidar.launch 예제:**

```xml
<?xml version="1.0"?>
<launch>
  <!-- Gazebo 시작 -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find your_package)/worlds/empty.world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
  </include>

  <!-- URDF 로드 -->
  <param name="robot_description" 
         command="$(find xacro)/xacro $(find your_package)/urdf/rplidar_c1_example.urdf"/>

  <!-- 로봇 상태 발행 -->
  <node name="robot_state_publisher" 
        pkg="robot_state_publisher" 
        type="robot_state_publisher"/>

  <!-- Gazebo 모델 생성 -->
  <node name="spawn_robot" 
        pkg="gazebo_ros" 
        type="spawn_model"
        args="-urdf -model robot -param robot_description"/>

  <!-- RViz 시각화 (선택사항) -->
  <node name="rviz" 
        pkg="rviz" 
        type="rviz" 
        args="-d $(find your_package)/config/rplidar.rviz"/>
</launch>
```

### 4.2 RViz 시각화

**rplidar.rviz 설정 예제:**

```yaml
Visualization Manager:
  - Class: rviz/RobotModel
    Enabled: true
    Links:
      base_link: true
      rplidar_c1_base: true
      rplidar_c1_laser: true

  - Class: rviz/LaserScan
    Enabled: true
    Topic: /scan
    Style: Billboards
    Color: 255; 0; 0
```

---

## 5. 실제 하드웨어 연동

### 5.1 ROS 드라이버 설치

```bash
sudo apt-get install ros-$(rosversion -d)-rplidar-ros
```

### 5.2 센서 연결

1. **USB 어댑터를 통한 연결**
   - RPlidar C1 → USB 어댑터 → 컴퓨터

2. **연결 확인**
   ```bash
   ls -l /dev/ttyUSB*
   ```

### 5.3 ROS Launch 파일

**rplidar_publisher.launch:**

```xml
<?xml version="1.0"?>
<launch>
  <node name="rplidar" pkg="rplidar_ros" type="rplidar_node">
    <param name="serial_port" value="/dev/ttyUSB0"/>
    <param name="serial_baudrate" value="115200"/>
    <param name="frame_id" value="rplidar_c1_laser"/>
    <param name="inverted" value="false"/>
    <param name="angle_compensate" value="true"/>
  </node>

  <!-- TF 브로드캐스터 -->
  <node name="tf_broadcaster" pkg="tf" type="static_transform_publisher"
        args="0 0 0 0 0 0 base_link rplidar_c1_laser 10"/>
</launch>
```

---

## 6. URDF 파일 검증

### 6.1 URDF 문법 확인

```bash
# URDF 파일이 유효한지 확인
check_urdf rplidar_c1.urdf

# 콘솔에 나무 구조 출력
urdf_to_graphviz rplidar_c1.urdf
```

### 6.2 Xacro를 이용한 생성

URDF에 변수를 사용하려면 Xacro를 사용하세요:

```bash
# Xacro URDF로 변환
rosrun xacro xacro rplidar_c1.urdf.xacro > rplidar_c1.urdf

# 파라미터 확인
rosrun xacro xacro rplidar_c1.urdf.xacro -p
```

---

## 7. 커스터마이징

### 7.1 센서 위치 변경

다음 부분에서 원점을 조정하세요:

```xml
<!-- 로봇 상단 중앙이 아닌 다른 위치에 마운트하려면 -->
<origin xyz="0 0 0.065" rpy="0 0 0"/>
              ↑ ↑ ↑
              x y z
```

### 7.2 회전 축 추가 (Continuous Joint)

실제 센서 회전 시뮬레이션을 위해:

```xml
<joint name="rplidar_rotation" type="continuous">
  <parent link="rplidar_c1_base"/>
  <child link="rplidar_c1_rotating"/>
  <axis xyz="0 0 1"/>
  <!-- 제한사항 없음 (continuous) -->
</joint>
```

---

## 8. 트러블슈팅

### 센서가 RViz에 보이지 않음
- ✓ robot_description 파라미터 확인
- ✓ TF 프레임 설정 확인
- ✓ URDF 문법 검증 (`check_urdf`)

### 스캔 데이터가 나오지 않음
- ✓ 포트 권한 확인: `sudo usermod -a -G dialout $USER`
- ✓ 연결 확인: `ls -l /dev/ttyUSB*`
- ✓ 드라이버 로그 확인: `roslaunch rplidar_ros rplidar.launch`

### 관성 오류
- ✓ 관성 행렬이 양수인지 확인
- ✓ 질량이 0이 아닌지 확인

---

## 9. 참고 자료

- [RPlidar C1 공식 위키](https://www.waveshare.com/wiki/RPLIDAR_C1)
- [ROS URDF 튜토리얼](http://wiki.ros.org/urdf)
- [ROS rplidar_ros 드라이버](http://wiki.ros.org/rplidar_ros)
- [Gazebo 시뮬레이션](http://gazebosim.org/)

---

## 10. 파일 설명

### 생성된 파일들

1. **rplidar_c1.urdf**
   - 센서 단독 URDF 모델
   - 최소 구성으로 센서 기하학적 정보만 포함

2. **rplidar_c1_example.urdf**
   - 로봇에 부착된 센서 예제
   - 로봇 베이스와 휠, 마운팅 브래킷 포함

3. **RPlidar_C1_Guide.md**
   - 이 문서
   - 사용 가이드 및 설정 방법

---

**작성일**: 2026-07-07  
**RPlidar C1 버전**: C1 (DTOF LiDAR)
