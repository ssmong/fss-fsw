# 004. fss_actuators — 밸브 드라이버

**상태: 완료**
**패키지 유형:** `ament_cmake` (C++17)

## 개요

GNC의 노즐별 목표 추력[N]을 입력↔추력 맵으로 역변환하고, 활성 상태와 최소 입력을 확인한 후
HAL을 통해 PLC에 전송하는 ROS2 노드.
메인 밸브 제어 서비스와 PLC watchdog heartbeat도 담당한다.

## 노드: valve_driver_node

### ROS2 인터페이스

| 유형 | 이름 | 타입 | 설명 |
|------|------|------|------|
| 구독 | `/actuators/thrust_commands` | `ThrustCommands` | 노즐별 목표 추력 [N] |
| 발행 | `/actuators/valve_commands` | `ValveCommands` | HAL 전송에 성공한 정규화 입력 (SIL/관측용) |
| 구독 | `/system/mode` | `SystemMode` | 안전 게이팅 (ACTIVE에서만 추력, E-STOP 시 HAL 비상정지) |
| 발행 | `/actuators/valve_status` | `ValveStatus` | 실제 적용 상태 발행 |
| 서비스 | `/actuators/main_valve` | `MainValveControl` | 에어베어링 on/off |

### 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `use_sim` | `false` | 시뮬레이션 모드 |
| `use_thrusters` | `true` | 런치 실행 플래그; false이면 추력 출력 0, 메인밸브 서비스 유지; read-only |
| `plc.ip` | `192.168.1.2` | PLC IP |
| `plc.port` | `502` | Modbus 포트 |
| `plc.timeout_sec` | `1.0` | 연결 타임아웃 |
| `plc.valve_register_start` | `100` | 비례밸브 시작 레지스터 |
| `plc.valve_count` | `8` | 밸브 수 |
| `plc.main_valve_register` | `99` | 메인 밸브 레지스터 |
| `plc.value_range` | `[0, 4000]` | 출력 범위 |
| `plc.deadband_minimum` | `0` | 이 값 미만의 입력 차단 (기본 비활성) |
| `plc.control_rate_hz` | `20.0` | 제어 주파수 |
| `thrust_map_file` | 빈 문자열 | 목표 추력 역변환 및 정적 추정 맵 (`valve_thrust_map_file`에서 공유) |
| `max_thrust_per_nozzle` | `${vehicle.max_thrust_per_nozzle}` | 맵 미사용 시 선형 역변환의 최대 추력 [N] |

`use_thrusters:=false`일 때 부팅 출력 클리어 후 모든 추력 명령은 0으로 전송한다.
PLC 연결, 메인밸브 서비스와 heartbeat, 상태 발행은 유지한다. 이 값은 기동 시 선택하며
런타임 파라미터 변경으로 다시 켤 수 없다. 전체 런치의
[장치 선택 옵션](../infra/011_Bringup.md#런치-인자)을 참고한다.

## 입력과 추력 맵

`/actuators/thrust_commands`의 `thrust_n`은 노즐별 목표 추력[N]이다.
**드라이버가 목표 추력→밸브 입력 역변환을 한 번 수행**한다. 맵이 없으면
`input = clamp(thrust_n / max_thrust_per_nozzle, 0, 1)`의 선형 관계를 사용한다.
음수는 0, 최대 추력 초과는 포화하며 NaN/Inf는 해당 채널을 0으로 처리한다.
변환된 입력에 개별 enable, `use_thrusters`, 데드밴드, ACTIVE 모드 조건을 적용한다.
HAL은 이 입력을 PLC 정수 값 `int(input * 4000)`으로 바꿔 전송한다.

HAL 전송 성공 후 `/actuators/valve_commands`에 같은 정규화 입력을 발행한다.
SIL 플랜트는 이 출력을 구독하므로 드라이버의 맵 변환과 출력 차단을 함께 검증한다.
HAL 전송 실패 시 새 적용 명령을 발행하지 않으며, 모드 전환으로 출력이 차단되면 0을 발행한다.
`ValveStatus.commanded_duty_cycles`는 역변환 후 안전 게이팅 전 입력이며,
`estimated_thrust`는 적용된 PLC 정수 입력으로 계산한 정적 추정값이다. 추력 센서 피드백은 아니다.
[실측 맵과 설정 방법](../ops/010_Simulation.md#밸브별-입력추력-맵)을 참고한다.

**인터페이스 변경:** GNC 및 외부 명령 송신자는 `ThrustCommands`를 발행해야 한다.
기존 `/actuators/valve_commands`는 이제 드라이버의 출력이며, PLC 명령 입력으로 구독하지 않는다.
새 메시지 생성과 적용을 위해 `fss_interfaces` 및 관련 패키지를 함께 다시 빌드한다.

## 기동 초기화

- `connect()` 직후 `emergency_stop()`을 1회 호출해 잔류 PLC 레지스터(D00100–D00107)를 0으로
  리셋한다 — 직전 세션이 비정상 종료(SIGKILL 등)됐을 때 잔류 추력값이 남는 것을 방지. 실패 시 기동 중단.

## Heartbeat / Watchdog

- 제어 주기(기본 50ms = 20Hz)마다 `D00099=1` 재전송(에어베어링 활성 시)
- 매 heartbeat마다 `valve_status`를 재발행 — 명령 유무와 무관하게 매니저가 액추에이터 생존/RTT를
  상시 관측(ARM 전 준비도 판정에 사용)
- PLC 래더에서 Modbus 수신 타임아웃 감지 → 비상 정지 (향후 원호 연구원과 래더 로직 추가 필요)

## 종료 처리

`destroy_node()` 호출 시 `hal.disconnect()` → 자동 비상 정지 → PLC 연결 해제.

## 실행

```bash
# Hydra 설정을 적용하는 밸브 런치 (시뮬레이션)
ros2 launch fss_bringup actuators.launch.py use_sim:=true
```

## 공압 부품 사양

### 비례 솔레노이드 밸브: SMC PVQ33-5G-16-01N-X33

추력 노즐 제어용 비례밸브 (8개). PLC 아날로그 출력(XBF-DC04A, 4–20mA)으로 구동.

| 항목 | 사양 |
|------|------|
| 시리즈 | PVQ3※-X33 (전류 드라이버 내장 특주품) |
| 타입 | N.C. (상시 폐) |
| 오리피스 | φ1.6mm (형번 16) |
| 포트 | 1/8" NPT |
| 입력 | 4–20mA (DC24V) |
| 최대 작동 압력차 | 0.7 MPa (≈101.5 psi) — 80psi 공급은 정격 내 |
| 최대 유량 | ~100 L/min (at 0.35 MPa) |
| 소비 전력 | 0–4W |
| 히스테리시스 | ≤10% (최고 동작 압력차 시) |
| 반복 정밀도 | ≤3% |
| 스타트 전류값 | ≤50% (= 12mA = PLC 2000) |

**유량 곡선 특성 (0.35 MPa):**
- 4 mA → 0 flow (폐쇄)
- ~8 mA → 유동 시작 (PLC값 ≈ 1000)
- 20 mA → ~100 L/min (전개)
- 개방 곡선과 폐쇄 곡선이 다름 → Phase 5 양방향 스윕 측정 필요

### 메인 솔레노이드 밸브: SMC VDW20KA

에어베어링 부유 제어용 온/오프 밸브. PLC 디지털 출력(P27, D00099)으로 제어.

| 항목 | 사양 |
|------|------|
| 시리즈 | VDW, size 2 |
| 타입 | N.C., 2-포트 직동식 |
| 최대 시스템 압력 | 2.0 MPa |
| 오리피스 | 2.3mm |
| 유량 특성 | C=0.58, b=0.45, Cv=0.18 |
| 전력/방수 | 3W, DC24V, IP65 |

### 추력 추정 (데이터시트 기반 역산)

**이론 초킹 유동 (0.55 MPa gauge, φ1.6mm 노즐):**
- P₀ = 0.65 MPa abs, A = 2.01e-6 m², 압력비 0.155 < 임계값 0.528 → 초킹
- v_exit = 313 m/s, ṁ = 3.1e-3 kg/s → **F ≈ 1.46 N/노즐** (밸브 손실 무시)

**데이터시트 유량 기반 (100 L/min at 0.7MPa, 0.55MPa 보정 → 89 L/min):**
- ṁ = 1.81e-3 kg/s → **F ≈ 0.57 N/노즐** (밸브 손실 반영)

**운용 추정값: ~1.0 N/노즐** (Phase 5 캘리브레이션에서 실측 교체)

### Jetson ↔ PLC ↔ 밸브 역할 분담

```
[Jetson FSW - valve_driver_node]
  역할: 추력→유량 역산, 히스테리시스 보상, 데드밴드 처리
  출력: PLC 명령값 (0–4000)
        ↓ Modbus TCP (FC16, ~2ms RTT)
[PLC - XBF-DC04A]
  역할: 명령값 → 4–20mA 전류 변환 (단순 DAC)
  래더: 원호 연구원 작성, 수정 금지
        ↓ 4–20mA 전류
[SMC PVQ33]
  역할: 전류 → 유량 → 추력 (비선형, 히스테리시스 발생)
```

### 공압 시스템 다이어그램

참조: [Diagram_pneumatic_system.jpg](Diagram_pneumatic_system.jpg)

```
에어 탱크 (77ci, 4500psi)
  → 1차 레귤레이터 → 800psi
    ├─ 추력 라인: 80psi → PVQ33 × 8 → 노즐 8개
    └─ 부유 라인: 60psi → VDW20KA → 에어베어링 3개
```

⚠ **주의**: 다이어그램은 설계도이며 실제 배관과 차이가 있을 수 있음.
2차 레귤레이터(60/80psi 분기)가 실물에서 미확인됨 (2026-04-01 점검).
1차 레귤레이터 출력(~80psi)이 직접 분기되는 것으로 추정.

## 의존성

- `fss_interfaces` (ThrustCommands, ValveCommands, ValveStatus, MainValveControl)
- `fss_hal` (`ValveHal`, `PlcHal`, `SimHal`)
- `rclcpp`, `ament_index_cpp`, yaml-cpp, nlohmann/json

## 반응휠: SAMD21

`reaction_wheel_driver_node`는 `ReactionWheelHal`을 통해 SAMD21 반응휠을 제어한다.
설정은 `config/hardware/reaction_wheel.yaml`에서 읽으며 MCU 펌웨어는
`torque_current_ctrl_simple` 기준이다. 현재 GNC의 QP 할당은 추력기를 사용하고,
반응휠은 별도 bench 런치에서 운용한다.

| 구분 | 현재 계약 |
|------|-----------|
| 제어 | 1kHz 순수 current PI, 20kHz sign-magnitude PWM |
| 명령 | `T,<wheel_torque_Nm>`, `T,0`, `Z/z`, `STOP` |
| 상태 | STOP=0, ACTIVE=1, FAULT=2 |
| telemetry | `S,<current_A>,<duty>,<KT×current_Nm>,<state>` (100Hz) |
| 응답 | `ACK,T,...`, `ACK,STOP,...`, `FAULT,...` |
| 전류 | motor + lead INA226, 연속 0.96A, hard trip 1.20A |
| 최대 wheel 토크 | 0.013488N·m |

### ROS 인터페이스

| 경로 | 타입 | 역할 |
|---|---|---|
| `/reaction_wheel/torque_cmd` | `std_msgs/Float64` | 휠 토크 [N·m], host 수신 시각으로 명령 나이 계산 |
| `/reaction_wheel/status` | `std_msgs/String` | JSON: 출력 허용·연결·fault·STOP 확인·telemetry 나이와 측정값 |
| `/reaction_wheel/enable` | `std_srvs/SetBool` | `true`: 새 STOP 상태 확인 후 출력 허용, `false`: STOP 전송 |

노드는 시작 시 출력을 비활성화한다. enable 이후 받은 명령만 20Hz로 전송한다.
명령 250ms·telemetry 100ms 미수신, 잘못된 입력 또는 FAULT가 발생하면 STOP을 보내고
출력을 비활성화한다. 복구 후 다시 enable해야 한다. USB 수신과 ROS timer는 기본
단일 스레드 executor에서 실행한다.

### 실행

```bash
# 모의 HAL로 노드 실행
ros2 launch fss_bringup reaction_wheel.launch.py

# 별도 터미널: 명령 스트림 시작
ros2 topic pub -r 20 /reaction_wheel/torque_cmd std_msgs/msg/Float64 '{data: 0.002}'

# 상태의 stop_confirmed=true 확인 후 출력 허용
ros2 service call /reaction_wheel/enable std_srvs/srv/SetBool '{data: true}'

# 출력 정지
ros2 service call /reaction_wheel/enable std_srvs/srv/SetBool '{data: false}'
```

실장치는 YAML의 `enabled: true`와 USB `port`를 설정하고
`ros2 launch fss_bringup reaction_wheel.launch.py dry_run:=false`로 실행한다.
`allow_gnc_commands: false`는 유지한다. 본체 양의 Z 토크와 휠 토크의 부호는 반대다.

현재 telemetry에는 휠 속도가 없다. host의 fault 유지·비유한 값 차단을 추가했으며,
MCU 자체의 fault 유지·비유한 전류 처리·과속 보호 구현 상태는 YAML에 기록한다.
