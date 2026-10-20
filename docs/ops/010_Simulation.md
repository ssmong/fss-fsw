# 010. fss_sim — SIL 시뮬레이션

**상태: 완료 (Phase 7b)**
**패키지 유형:** `ament_python`

## 목적

Software-in-the-Loop(SIL) 시뮬레이션을 위한 플랜트 모델.
실제 하드웨어 없이 GNC 파이프라인을 검증한다.
밸브 명령을 ROS2 토픽으로 수신하여 3-DOF 동역학을 시뮬레이션하고,
가상 센서 데이터를 발행한다.

`sim.launch.py use_arm:=true use_thrusters:=false`로 팔을 포함한 무추력 구성을 실행한다.
`use_thrusters`는 기본 true인 read-only 실행 플래그다. false이면 외부 밸브 명령을
받아도 추력에 의한 힘·모멘트는 0이며, 센서·시계·동역학 전파는 계속 실행한다.
팔과 본체의 결합 동역학은 포함하지 않는다.

## 모듈 구조

```
fss_sim/
├── __init__.py
├── plant_model_node.py      # ROS2 노드 (1000Hz 내부 루프 + 센서 발행)
├── dynamics.py              # 3-DOF 평면 동역학 (RK4 적분)
├── catalog_curve.py         # PVQ30 곡선 검증 + 전류/압력 보간
├── valve_model.py           # ideal/real/mapped 밸브 모델
└── sensor_model.py          # 가상 센서 노이즈 모델 (Vicon + IMU)
```

## 동역학 모델 (`PlanarDynamics`)

3-DOF 평면 (x, y, θ) 뉴턴-오일러 방정식 (본체 프레임 진실, transport 항 포함):

```
m·v̇x  = ΣFx + m·ω·vy   (본체 프레임 속도이므로 transport 항 −ω×v 포함)
m·v̇y  = ΣFy − m·ω·vx
Izz·ω̇ = ΣMz
```

- 상태 벡터: `[px, py, θ, vx, vy, ω]` (6D, **속도는 본체 프레임 — 플랜트 내부 진실**)
- **적분: RK4** (4차 Runge-Kutta)
- 위치 미분은 본체→테이블 회전 `ṗ = R(θ)·v_body` 포함
- 발행 시점에 속도를 R(θ)로 테이블(관성) 프레임으로 회전 (`plant_model_node`)
  → EKF/컨트롤러의 관성 프레임 규약과 정합
- 각도 정규화: [-π, π]
- 에어베어링 마찰: 무시 (이상적 모델)

기본값:
- 질량: 10.0 kg (`config/vehicle/mass_properties.yaml`)
- 관성: 0.5 kg·m² (`config/vehicle/mass_properties.yaml`)
- 추력기 기하학: `config/vehicle/thruster_geometry.yaml` → B 행렬 (3×8)

## 밸브 모델 (`ValveModel`)

Hydra 그룹 `system/sim_valve`로 모델을 선택한다. 기본값은 `real`이다.

| 프로필 | 용도 | 명령→추력 경로 |
|--------|------|----------------|
| `ideal` | 제어기·할당기 기준 성능 확인 | 즉시 선형 변환 `F = Fmax·u` |
| `real` | PVQ30/PVQ33 카탈로그 기준선 | X33 전류 변환 → 압력별 상승·하강 LUT → 선택적 확인된 동특성 |
| `mapped` | 밸브별 정적 추력 맵 | 입력→추력[N] 구간 선형 보간 → 선택적 지연·1차 응답 |

### 밸브별 입력↔추력 맵

```bash
python scripts/run_experiment.py experiment=valve_map_demo
```

[밸브 맵](../../config/calibration/valve_thrust_measured.yaml)은 **실측 기반 정적 맵**이다.
8개 밸브 모두 데드존 없이 `추력[N] = 0.5 × 입력`으로 선형 증가하며 전개방 추력은 0.5N이다.
`duty`는 공통 입력 축(0~1), `thrust_n`은 D00100~D00107 순서의 추력[N] 행이다.
PLC 값은 `int(duty * 4000)`이다.

- GNC는 맵의 노즐별 최대 추력으로 할당하고 `ThrustCommands.thrust_n` [N]을 발행한다.
- 밸브 드라이버는 목표 추력을 역보간해 입력으로 변환하고, 출력 차단 조건을 적용해 HAL로 보낸다.
- HAL 전송 성공 후 드라이버가 발행한 `ValveCommands`를 플랜트가 수신한다.
- 드라이버는 적용된 PLC 정수 값으로 `estimated_thrust`를 계산하며, `mapped` 플랜트는 같은 맵으로 추력을 계산한다.
- 0N 명령은 입력 0, 범위 밖 목표 추력은 끝값으로 제한한다.

맵은 `valve_thrust_map_file` 한 곳에서 지정하며 경로는 `config/` 기준이다.
맵을 갱신할 때도 같은 형식으로 파일을 바꾸면 된다. 입력 축은 0~1 순증가,
추력은 0에서 시작하는 비감소 곡선이어야 한다. 맵 선택 시 최대 추력은 파일의 N 값을 사용한다.

```bash
# 실측 정적 맵 사용
python scripts/run_experiment.py valve_thrust_map_file=calibration/valve_thrust_measured.yaml
```

빈 경로(기본값)는 기존 선형 할당을 유지하고 추정 추력은 0으로 표시한다.
정적 맵은 고정 공급압 기준이며 압력 변화·히스테리시스는 포함하지 않는다.
`input_delay_ms`, `tau_valve_ms`는 mapped 플랜트에서도 별도로 설정할 수 있다.

### 카탈로그 모델

`real`은 [디지타이즈 LUT](../../config/system/sim_valve/pvq33_5g_16_x33_flow_curve.yaml)의
φ1.6 곡선 네 쌍을 직접 사용한다. 전류 증가 시 오른쪽 곡선, 감소 시 왼쪽 곡선을
선택하고 데이터점을 정확히 통과하는 단조 PCHIP으로 연결한다. 80psi(g)는
0.55158MPa이므로 0.5/0.7MPa 곡선의 같은 전류 유량을 압력축에서 보간한다.

X33 축 변환과 출력은 다음과 같다.

```text
i_X33 [mA] = 4 + 16u
i_coil,equivalent [mA] = 10 i_X33
Q = LUT(direction, i_coil,equivalent, ΔP)
F = F_reference · Q / Q_reference(0.7 MPa)
```

`×10`은 SMC X33 전용 φ2.3 곡선의 활성 구간(약 6–16.5mA)이 표준 DC24V
코일 곡선의 약 60–165mA 구간과 같은 형상인 점에서 얻은 전류축 축척이다.
따라서 4–20mA 전체를 0–165mA에 맞추는 변환은 사용하지 않는다.
`Q`의 형상과 압력 의존성은 카탈로그 기반이며, 낮은 압력의 최대 유량을 다시 1로
정규화하지 않는다. 따라서 80psi에서 전개방 유량은 약 86.737L/min으로,
0.7MPa 기준 109L/min보다 작다. `F_reference`는
`vehicle.max_thrust_per_nozzle`의 미보정 절대 추력 스케일이다. 밸브 유량 그래프만으로
노즐 추력을 확정할 수 없으므로 Phase 5 가속도 실측 전에는 두 근거를 섞지 않는다.

### `real` 기본값과 근거

| 파라미터 | 값 | 근거 |
|---------|---:|------|
| `catalog_curve_file` | `pvq33_5g_16_x33_flow_curve.yaml` | 병합 PDF 2쪽의 φ1.6 major loop |
| `pressure_diff_mpa` | 0.55158 | 80psi(g), 출구 대기압 |
| `signal_current_min_ma` | 4.0 | X33 입력 사양 |
| `signal_current_max_ma` | 20.0 | X33 입력 사양 |
| `signal_to_coil_current_gain` | 10.0 | X33·표준 곡선의 형상 보존 전류축 축척 |
| `input_delay_ms` | 0.0 | 확인된 지연값 없음 |
| `tau_valve_ms` | 0.0 | 응답시간 수치 없음 |

SMC PVQ33-5G-16-01N-X33 데이터시트는
2쪽에서 압력별 상승·하강 곡선을 제공한다. `히스테리시스 ≤10%`, 반복 정밀도 `≤3%`,
스타트 전류 `≤50%`는 제품 보증 한계이므로 모델값으로 사용하지 않는다. 카탈로그에는
minor loop와 시간 응답이 없으므로 `real`은 major loop만 모델링하고 지연·시상수는
0으로 둔다. 확인된 측정값이 생긴 경우에만 해당 파라미터를 덮어쓴다.

## 센서 모델 (`sensor_model.py`)

### Vicon 모델 (MX T10)

| 항목 | 노이즈 |
|------|--------|
| 위치 (x, y) | N(0, 5e-4 m) — 0.5mm σ |
| 방향 (θ) | N(0, 1.745e-3 rad) — ~0.1° σ |

### IMU 모델 (MTi-630-DK)

| 항목 | 값 | 설명 |
|------|-----|------|
| 자이로 백색 노이즈 | 1.2217e-4 rad/s/√Hz | ARW (데이터시트 기반) |
| 가속도 백색 노이즈 | 5.886e-4 m/s²/√Hz | VRW (데이터시트 기반) |
| 자이로 바이어스 랜덤 워크 | 1e-5 rad/s/√s | 바이어스 불안정성 |
| 가속도 바이어스 랜덤 워크 | 1e-4 m/s²/√s | 바이어스 불안정성 |

- 노이즈 파라미터는 `config/gnc/navigator/ekf.yaml`과 일관성 유지
- IMU 출력: (ω_z, a_x, a_y) + 바이어스 + 백색 노이즈

## ROS2 인터페이스

| 유형 | 이름 | 타입 | 주기 | 설명 |
|------|------|------|------|------|
| 구독 | `/actuators/valve_commands` | `ValveCommands` | — | 드라이버가 변환·게이팅한 8개 밸브 입력 (ZOH) |
| 발행 | `/sensors/vicon_state` | `SpacecraftState` | 100Hz (설정 가능) | 노이즈 포함 위치/자세 |
| 발행 | `/sensors/imu` | `sensor_msgs/Imu` | 100Hz (설정 가능) | 노이즈 + 바이어스 포함 IMU |
| 발행 | `/sim/true_state` | `SpacecraftState` | Vicon 주기 | 노이즈 없는 참값 (EKF 검증용) |
| 발행 | `/clock` | `rosgraph_msgs/Clock` | sim 스텝 | sim time 소스 |

- **ZOH (Zero-Order Hold)**: 마지막 수신된 밸브 명령을 다음 수신까지 유지 (실제 PLC 동작과 동일)
- **내부 루프**: 1000Hz (고정밀 적분), 센서 발행 주파수는 독립 설정
- **sim time**: 플랜트가 `/clock`을 발행하고 EKF/GNC는 `use_sim_time:=true`로 이를 따른다
  (플랜트 자신은 wall-clock 구동 — 타이머 지연 시에도 물리·항법 dt 정합)

## 파라미터

```yaml
plant_model_node:
  ros__parameters:
    mass: 10.0
    inertia_zz: 0.5
    initial_x: 0.5
    initial_y: 0.3
    initial_theta: 0.2
    max_thrust_per_valve: 1.0
    valve_model_mode: real
    catalog_curve_file: system/sim_valve/pvq33_5g_16_x33_flow_curve.yaml
    pressure_diff_mpa: 0.55158
    signal_current_min_ma: 4.0
    signal_current_max_ma: 20.0
    signal_to_coil_current_gain: 10.0
    input_delay_ms: 0.0
    tau_valve_ms: 0.0
    vicon_rate_hz: 100.0
    imu_rate_hz: 100.0
    sim_rate_hz: 1000.0
    vicon_position_noise: 5.0e-4
    vicon_heading_noise: 1.745e-3
    gyro_noise_density: 1.2217e-4
    accel_noise_density: 5.886e-4
    gyro_bias_random_walk: 1.0e-5
    accel_bias_random_walk: 1.0e-4
```

## 실행 방법

```bash
# 현실 밸브 모델 (기본)
ros2 launch fss_bringup sim.launch.py

# 이상 밸브 모델
FSS_HYDRA_OVERRIDES="system/sim_valve=ideal" \
  ros2 launch fss_bringup sim.launch.py

# 실험 러너 사용 시
python scripts/run_experiment.py system.use_sim=true system/sim_valve=ideal
```

카탈로그 LUT와 80psi 보간 곡선을 X33 4–20mA 축으로 확인하려면:

```bash
python scripts/plot_valve_catalog.py
python scripts/plot_valve_catalog.py --output valve_curve.png
```

## 데이터 흐름

```
valve_commands → ValveModel (ideal 또는 X33 축 변환+카탈로그 major loop+압력 보간)
  → B 행렬 (8 추력 → Fx, Fy, Mz)
  → PlanarDynamics (RK4 @ 1000Hz)
  → state [px, py, θ, vx, vy, ω]
  → SensorModel (Vicon 노이즈 + IMU 바이어스/노이즈)
  → /sensors/vicon_state + /sensors/imu
```

## 테스트

단위 테스트 (`tests/test_sim.py`):
- 동역학: 자유 비행, 추력 응답, 각도 래핑
- 밸브: ideal 즉시 응답, 공식 앵커값, X33 1:10 축척, 압력 보간,
  상승·하강 major loop, 선택적 지연·1차 응답
- 센서: 노이즈 통계, 바이어스 랜덤 워크

설치된 C++ 런타임과 Python 플랜트를 함께 실행하는 `tests/test_cxx_runtime.py`:
- mapped 원점 수렴: 30초 후 위치 오차 < 0.05 m
- real 원점 수렴: 45초 후 위치 오차 < 0.08 m
- real 비원점 목표 수렴: 30초 후 위치 오차 < 0.05 m
- 모든 수렴 검사에서 방향 오차 < 0.1 rad, 실제 ROS 토픽·서비스와 E-stop 확인

## Foxglove 모니터링

SIL 전용 Foxglove 레이아웃: `config/foxglove/sil_ekf_tuning.json`
- 위치/자세: 참값(`/sim/true_state`) vs EKF 추정(`/navigation/state`) 오버레이
- User Script로 계산한 EKF 추정 오차와 GNC 추종 오차
- EKF 위치 불확실도, 참값/추정 속도, 명령/적용 밸브 값, 시스템 로그

```bash
ros2 launch fss_bringup sim.launch.py          # SIL 실행
bash scripts/launch/start_foxglove.sh          # 브릿지 시작
# 노트북: ssh -L 8765:localhost:8765 → Foxglove ws://localhost:8765
```

## 향후 작업

- Phase 5 밸브별 정적 상승/하강 곡선으로 카탈로그 major-loop LUT 교체
- 점성 마찰 모델 옵션 추가
