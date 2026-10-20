# 005. fss_sensors — 센서 래퍼

**상태: 완료**
**패키지 유형:** `ament_cmake` (C++17 런타임, Python 보정 도구)

## 개요

외부 센서 시스템(Vicon 모션 캡처, Xsens MTi-630-DK IMU)의 데이터를
FSS 인터페이스로 변환하여 항법 모듈에 통일된 센서 데이터를 제공한다.

**RealSense D435i**는 영상·깊이 알고리즘과 보조 IMU 기록용이며, 현재 상태 추정에는
사용하지 않는다.

## 모듈 구조

```
fss_sensors/
├── src/vicon_bridge_node.cpp  # UDP 평면 포즈 → SpacecraftState
├── src/imu_bridge_node.cpp    # Xsens IMU 좌표계 변환
└── fss_sensors/               # 런타임과 분리된 Python 보정 도구
    ├── imu_transform.py       # 보정용 회전·쿼터니언 수학
    ├── imu_calibration.py     # IMU 정지 통계와 품질 판정
    └── float_calibration.py   # Vicon 부상 trim·드리프트 계산
```

## 센서 구성

| 센서 | 모델 | 인터페이스 | 용도 |
|------|------|-----------|------|
| Vicon 카메라 | MX T10 | WiFi (지상국 → Jetson UDP 9870) | 1차 위치/자세 추정 |
| IMU | Xsens MTi-630-DK (AHRS) | USB (UART-to-USB) | EKF 융합 보정 |
| 비전/깊이/IMU | Intel RealSense D435i | USB 3.x | 별도 항법 테스트·기록 |

## vicon_bridge_node

Vicon 지상국 프로그램 FSS_VICON이 계산한
테이블 위치·방향각을 UDP JSON으로 받는다. Jetson에는 Vicon SDK가 필요 없다.

```text
Nexus → 지상국 FSS_VICON → UDP JSON → ViconHAL → 브릿지 → /sensors/vicon_state
```

### 연결

1. 지상국에서 추적할 물체를 등록한다.
2. FSS_VICON의 UDP 목적지를 **Jetson IP**, 포트를 **9870**으로 설정하고 송신을 켠다.
3. `config/hardware/vicon.yaml`의 `object_name`을 등록한 이름과 맞춘다. 기본값은 `robotA`다.
4. Jetson에서 `ros2 launch fss_bringup sensors.launch.py`로 실행한다.

| 설정 | 기본값 | 의미 |
|---|---|---|
| `bind_ip` | `0.0.0.0` | Jetson에서 수신할 인터페이스 |
| `port` | `9870` | 수신 UDP 포트 |
| `source_ip` | 빈 문자열 | 송신 IP 제한. 빈 값이면 제한 없음 |
| `object_name` | `robotA` | JSON `objects`에서 선택할 물체 이름 |
| `poll_rate_hz` | `200.0` | 비차단 소켓 폴링 주기 |
| `vicon_frame_id` | `table` | 발행할 테이블 좌표계 |
| `frame_offset_x/y` | `0.0` | 추가 테이블 위치 보정 [m] |
| `frame_rotation_deg` | `0.0` | 추가 테이블 축 회전 [deg] |

### 패킷과 처리

프로토콜 기준: FSS_VICON `vt/udp.py` (51eb64f).
한 UDP datagram이 한 프레임이다.

```json
{"frame": 13183, "t": 1760512345.123,
 "objects": {"robotA": {"valid": true, "x": 1234.5, "y": 678.9, "yaw": 37.2, "n": 4, "err": 0.3}}}
```

- HAL이 x/y의 mm를 m로, 반시계 양의 yaw를 deg에서 rad로 변환한다.
- 입력은 이미 지상국에서 정한 테이블 좌표다. 같은 원점·축을 쓰면 추가 보정값은 0으로 둔다.
- 물체가 없거나 `valid=false`이면 발행하지 않는다. 비정상 숫자·손상 패킷·중복·역순도 폐기한다.
- 수신 큐를 비우고 최신 프레임만 사용한다. 최신 프레임에서 추적이 끊기면 앞선 유효 표본도 발행하지 않는다.
- `n`과 `err`는 지상국의 진단값이며 현재 EKF 입력에는 쓰지 않는다.
- `t`는 실제 장치에서 PC가 Vicon 프레임을 읽은 Unix 시각이다. 속도 차분·순서 확인에 사용한다.
  ROS `header.stamp`는 Jetson 수신 시각이며 PC와 시계 동기화가 필요 없다.
  네트워크 지연과 오래된 패킷의 절대 나이는 이 시각으로 측정하지 않는다.
- 지상국 `--sim`은 상대 시각을 보내지만 같은 수신기를 사용할 수 있다.
  송신 시각이 재시작으로 과거로 돌아가면 브릿지도 재시작한다.

발행 토픽은 `/sensors/vicon_state` (`SpacecraftState`, m·rad·m/s·rad/s)다.
**UDP에는 z·roll·pitch가 없으므로 6-DOF `/sensors/vicon_pose`는 발행하지 않는다.**

## imu_bridge_node

외부 Xsens ROS2 드라이버의 `sensor_msgs/Imu`를 구독하여
`XsensImuHAL`에 변환·검증을 맡긴 뒤 FSS 본체 프레임으로 재발행한다.
USB 통신은 외부 드라이버가, 장착 보정·포화 검사는 HAL이 담당한다.

### ROS2 인터페이스

| 유형 | 이름 | 타입 | 설명 |
|------|------|------|------|
| 구독 | `/imu/data` | `sensor_msgs/Imu` | Xsens 드라이버 출력 |
| 발행 | `/sensors/imu` | `sensor_msgs/Imu` | FSS 좌표계 IMU |

### 파라미터 (값 소스: `config/hardware/imu.yaml` — 필수, 기본값 없음)

| 파라미터 | 설정값 | 설명 |
|---------|--------|------|
| `input_topic` | `"/imu/data"` | 입력 IMU 토픽 (공식 드라이버 발행) |
| `output_topic` | `"/sensors/imu"` | 출력 IMU 토픽 |
| `frame_id` | `"body"` | 보정 후 출력 프레임 |
| `mounting_quaternion_xyzw` | `[0, 0, 0, 1]` | 센서 벡터를 body로 보내는 고정 회전 |

### 좌표계

```
/imu/data (imu_link, 센서 원본)
  → mounting_quaternion_xyzw로 gyro/accel/공분산 회전
  → orientation의 고정 장착 회전 제거
  → /sensors/imu (body, X 전방·Y 좌측·Z 상방)
```

`xsens_mti_ros2_driver` 출력은 우수좌표계 Z-up (수평 정지 시 `accel.z ≈ +9.81 m/s²`).
장착 방향은 단위 쿼터니언 하나로만 표현한다 — 축별 부호 반전은 사용하지 않는다.

EKF는 `angular_velocity.z`, `linear_acceleration.x/y`만 사용한다.
`orientation`은 진단용이며 roll/pitch는 3-DOF 상태에 포함하지 않는다.

## IMU 확인과 실제 부상 검사

IMU는 기체 중앙에 본체 축과 같은 방향으로 장착되어 있어 identity quaternion을 쓴다.
장착 quaternion은 자동 생성하지 않는다. 다음 반응만 확인하고, 축이 다를 때에만
`config/hardware/imu.yaml`을 수정한다.

- 본체 +X 방향으로 움직일 때 `accel.x`가 주로 반응하는지 확인
- 반시계 회전할 때 `angular_velocity.z > 0`인지 확인
- 정지 시 `accel.z ≈ +9.81 m/s²`인지 확인

### 1. 에어 OFF 정적 센서 검사

기체를 움직이지 않게 두고 센서 수신률, 중력 크기, gyro/accel 노이즈만 검사한다.

```bash
python3 scripts/calibration/imu_static_check.py \
  --output calibration/imu_static_check.yaml
```

수평축 평균은 리포트에 표시만 하고 합격 조건에는 쓰지 않는다.
받침 높이와 테이블 경사가 섞여 있으므로 장착 roll/pitch로 바꾸면 안 된다.

### 2. 에어 OFF/ON 부상 검사

현재 FSS_VICON UDP는 x/y/yaw만 제공하므로 높이·기울기 변화는 측정할 수 없다.
`vicon_float_check.py`는 별도 **실측 6-DOF PoseStamped** 입력이 있을 때만
`--topic`을 지정해 사용한다. 평면 위치에 z=0, roll=pitch=0을 채운 값으로 검사하면 안 된다.

### 입력 검증

- **NaN/Inf 검사**: 비정상 값 감지 시 메시지 폐기 + 경고 로그
- **센서 포화 검사**: 자이로 |ω| > 34 rad/s (MTi-630 ±2000°/s의 97%), 가속도 |a| > 190 m/s² (±20g의 97%) 초과 시 경고
- **설정 검사**: 장착 quaternion이 4개 유한값이며 정규화 가능한지 기동 시 확인

## MTi-630-DK 사양 (참조)

| 항목 | 값 |
|------|-----|
| 센서 유형 | AHRS (자이로+가속도계+지자기+기압계) |
| 자이로 노이즈 | 0.007 °/s/√Hz |
| 자이로 바이어스 안정성 | 8 °/h (in-run) |
| 가속도계 노이즈 | 60 μg/√Hz |
| 자세 정확도 | Roll/Pitch 0.2° static, Yaw 1° dynamic |
| 최대 출력 | 400 Hz |
| DK 인터페이스 | USB (UART-to-USB 변환기 내장) |

## 외부 의존성

| 패키지 | 설치 방법 | 용도 |
|--------|-----------|------|
| `xsens_mti_ros2_driver` | 소스 빌드 (공식) | MTi-630 USB 드라이버 |
| `ros-humble-realsense2-camera` | apt | D435i 영상·깊이·보조 IMU 드라이버 |

## 런치 파일

```bash
# 센서만 실행 (Vicon 접속 정보는 config/hardware/vicon.yaml — SSoT)
ros2 launch fss_bringup sensors.launch.py

# 전체 시스템 (센서 포함)
ros2 launch fss_bringup fss_full.launch.py

# D435i까지 선택 실행
ros2 launch fss_bringup fss_full.launch.py use_realsense:=true
```

## 설정 파일

- `config/hardware/imu.yaml` — MTi-630 연결 설정, 장착 quaternion, 출력 좌표계 (Hydra `# @package imu_bridge_node`)
- `config/hardware/vicon.yaml` — FSS_VICON UDP 수신 주소·포트·물체 이름·좌표 설정
  (Hydra `# @package vicon_bridge_node`)
- `config/hardware/realsense.yaml` — RealSense D435i 드라이버 설정. **Hydra 트리 밖**의
  realsense2_camera `config_file` 포맷(flat dict). `sensors.launch.py`가 `config_file:=`로
  로드하며 `start_realsense.sh` 기본 모드도 동일 파일 사용한다. ROS2 숫자 변환을 막는
  `_` 접두사를 붙인 SN(`_247122073297`)과 장치형(`d435i`)을 함께 검사한다.

### RealSense D435i 데이터 경로

D435i는 FSS 상태 추정(EKF)에 **미사용**이며, 브릿지 없이 realsense2_camera가 다음
토픽을 직접 발행한다.

- `/camera/camera/color/image_raw` — 컬러 (640×480@30)
- `/camera/camera/depth/image_rect_raw` — 깊이 (640×480@30)
- `/camera/camera/aligned_depth_to_color/image_raw` — 컬러 프레임에 정렬한 깊이
- `/camera/camera/imu` — 200Hz gyro 시각에 보간한 100Hz accel 포함 보조 IMU
- `/diagnostics` — 스트림 주기와 카메라 온도 (1초 주기)
- `/tf_static` — D435i 내부 센서 사이의 고정 변환

기본 설정: color/depth 동기화 + depth-to-color 정렬 ON, point cloud·RGBD 중복 토픽 OFF
(CPU·USB·rosbag 부하). 필요 시 `pointcloud.enable:=true`.

### Xsens와 D435i IMU의 경계

- EKF는 Xsens가 body 프레임으로 보정해 발행하는 `/sensors/imu`만 구독한다.
- D435i `/camera/camera/imu`를 `/sensors/imu`로 remap하지 않는다.
- D435i의 IMU와 영상은 같은 카메라 알고리즘의 시간 동기·기록 검증에만 사용한다.
- D435i 내부 TF는 드라이버가 발행하지만, `body→camera_link` 장착 TF는 위치·방향을
  실측하기 전까지 발행하지 않는다. VO/SLAM 통합 전에 이 외부 변환을 추가한다.

accel은 100Hz 프로파일. Jetson USB 3.x 포트에 직접 연결하고, 프레임 드롭 시 `low` 모드.
SN·USB 링크 확인: `rs-enumerate-devices -s`.

```bash
bash scripts/launch/start_realsense.sh       # 640×480@30 + 보조 IMU
bash scripts/launch/start_realsense.sh low   # 영상 15fps + 보조 IMU
bash scripts/launch/start_realsense.sh depth # 저해상도 depth만
```

`use_realsense:=true`일 때만 기동 (기본 false). 진단: `scripts/diagnostics/realsense_live.py`.
