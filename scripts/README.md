# FSS 스크립트 가이드

## 실행 순서

```bash
# 1. 시스템 점검
bash scripts/preflight_check.sh

# 2. Xsens 원시 드라이버 시작 (별도 터미널)
bash scripts/launch/start_imu.sh

# 3. 별도 터미널에서 전체 시스템 기동 (IDLE, IMU 브리지 포함)
ros2 launch fss_bringup fss_full.launch.py

# 4. 에어 OFF IMU 검사 후 모드 전환 절차 진행
python3 scripts/calibration/imu_static_check.py

# 5. 원격 Foxglove 모니터링
bash scripts/launch/start_foxglove.sh
```

`start_imu.sh`는 `/imu/data`를 발행하는 Xsens 드라이버만 실행한다. IMU 브리지는
전체 런치가 Hydra 설정으로 실행하므로 별도로 중복 실행하지 않는다.
전체 시스템 없이 센서만 확인할 때는 `sensors.launch.py`를 사용하고, 전체 런치 전 종료한다.
D435i는 전체 런치의 `use_realsense:=true` 또는 아래 독립 실행기 중 하나로 실행한다.

## Hydra 실험 실행

저장소 루트에서 실험 프리셋, 본체 SIL, 팔·추력기·GNC 사용 여부를 선택한다.

```bash
python scripts/run_experiment.py experiment=default
python scripts/run_experiment.py system.use_sim=true system.use_arm=true system.use_thrusters=false
python scripts/run_experiment.py system.use_sim=true system.use_gnc=false
python scripts/run_experiment.py experiment=thrust_characterization
```

장치 선택 기본값은 팔 OFF, 추력기 ON, GNC ON이다. `system.use_thrusters=false`이면
GNC도 실행하지 않으며 에어베어링 메인밸브 서비스는 유지한다.
팔 연결과 enable 절차는 [본체·로봇팔 운용](../docs/ops/013_Arm_Execution.md)을 따른다.
`thrust_characterization`은 추력기는 유지하면서 본체 GNC를 끄므로 외부에서 밸브 명령을 보낼 수 있다.
각 실험은 Ctrl+C로 직접 종료한다. `experiment.duration_sec`·`record_bag`는 현재 메타데이터이며
자동 종료·자동 기록에 적용되지 않는다. rosbag 기록은 별도 터미널에서 수동으로 시작·종료한다.

## IMU와 부상 상태 확인

IMU가 기체 중앙에 가깝고 축 표시가 본체 축과 같으므로 기본 장착 quaternion은 identity다.
별도 장착각을 추정하지 않고 실제 축 반응만 확인한다.

```bash
# 에어 OFF·기체 고정: IMU 수신률과 노이즈 검사
python3 scripts/calibration/imu_static_check.py \
  --output calibration/imu_static_check.yaml
```

지상국 FSS_VICON은 x/y/yaw만 보내므로 높이·기울기의 부상 검사는 지원하지 않는다.
`vicon_float_check.py`는 별도 실측 6-DOF PoseStamped 입력이 있을 때 `--topic`을 지정해 사용한다.
[센서 연결·패킷 설명](../docs/hardware/005_Sensors.md#vicon_bridge_node)을 참고한다.

## RealSense D435i

```bash
bash scripts/launch/start_realsense.sh       # 기본: 640×480@30, 정렬 depth, 보조 IMU
bash scripts/launch/start_realsense.sh low   # 영상 15fps, 보조 IMU 유지
bash scripts/launch/start_realsense.sh depth # depth만, color/IMU 비활성
```

D435i IMU는 카메라 알고리즘용 `/camera/camera/imu`에만 남겨 둔다. 현재 EKF 입력인 Xsens
`/sensors/imu`로 remap하지 않는다. 영상 확인은 `diagnostics/realsense_live.py`를 사용한다.

## 시각화 도구 선택

| 도구 | 추천 용도 | 실행 위치 |
| --- | --- | --- |
| Foxglove | 기본 원격 모니터링, 자유로운 plot 추가, rosbag 분석 | 노트북 브라우저/앱 |
| `live_plot.py` | 실험 관제 콘솔 — 모니터링 + 제어(E-STOP/모드 스테퍼/에어베어링) | ROS2 네트워크에 연결된 노트북 |
| `diagnostics/imu_live.py` | IMU raw/body 축 비교와 평면 yaw 확인 | 노트북 또는 Jetson GUI |
| `diagnostics/realsense_live.py` | Depth/Color 영상 확인 | 노트북 또는 Jetson GUI |

### 커스텀 미션 대시보드

초기 화면에는 모드, 안전 상태, XY 궤적, 목표와 오차만 표시합니다. 속도, EKF 불확실도, 밸브, 로그는 탭으로 분리되어 있습니다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 scripts/live_plot.py
python3 scripts/live_plot.py --history-sec 60 --fps 20
python3 scripts/live_plot.py --fullscreen
python3 scripts/live_plot.py --opengl       # PyOpenGL 설치 시 선택
```

제어 바의 모드 스테퍼(PRESSURIZE→READY→ACTIVE)는 FSM 허용 전이 + 서브시스템 준비도가 충족될 때만 활성화되고, E-STOP은 항상 활성입니다. 실제 게이팅은 `manager_node`가 강제하며 GUI는 이를 미러링합니다.

새 시계열은 `live_plot.py`의 `Dashboard._build_detail_tabs()`에서 `SeriesSpec`을 추가해 공통 스타일과 자동 갱신을 재사용합니다. `line_style`은 `solid`, `dashed`, `dotted` 중에서 선택할 수 있습니다.

### Foxglove

```bash
# Jetson
bash scripts/launch/start_foxglove.sh

# 노트북
ssh -L 8765:localhost:8765 <user>@<jetson-ip>
# Foxglove에서 ws://localhost:8765 연결
```

레이아웃은 `config/foxglove/`에 있습니다.

- `hardware_monitor.json`: 기본 운용 화면
- `sil_ekf_tuning.json`: SIL 및 EKF 튜닝
- `imu_debug.json`: IMU raw/body 비교

가져오기와 plot 추가 방법은 [Foxglove 레이아웃 가이드](../config/foxglove/README.md)를 참고합니다.

## 폴더 구조

```text
scripts/
├── calibration/
│   ├── imu_static_check.py     # 에어 OFF IMU 정적 품질 검사
│   └── vicon_float_check.py    # 외부 6-DOF 입력의 에어 OFF/ON 비교
├── diagnostics/
│   ├── arm_launch_smoke.py     # SIL·팔 dry_run 기동과 모드 전환 검사
│   ├── imu_live.py             # IMU raw/body 실시간 비교
│   └── realsense_live.py       # RealSense depth/color 뷰어
├── launch/
│   ├── start_foxglove.sh       # Foxglove Bridge
│   ├── start_imu.sh            # Xsens 원시 드라이버 (브리지는 전체/센서 런치)
│   └── start_realsense.sh      # RealSense 드라이버
├── compute_lqr_gains.py        # LQR 게인 계산
├── install_dependencies.sh     # 런타임/개발 의존성 설치
├── live_plot.py                # 메인 미션 대시보드
├── preflight_check.sh          # 하드웨어와 네트워크 사전 점검
├── run_experiment.py           # Hydra 실험 실행기
└── validate_config.py          # 설정과 노드 파라미터 정합 검증
```
