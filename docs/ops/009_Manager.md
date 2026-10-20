# 009. fss_manager — 시스템 모드 관리자

**상태: 완료 (Phase 7a)**
**패키지 유형:** `ament_cmake` (C++17)

## 목적

유한 상태 기계(FSM)로 시스템 운영 모드를 관리하고,
각 서브시스템의 건강 상태를 모니터링하여 안전한 운영을 보장한다.

## 모듈 구조

```
fss_manager/
├── include/fss_manager/     # FSM·건강 감시와 ROS 어댑터 선언
├── src/mode_fsm.cpp         # ROS 독립 FSM + 건강 감시
└── src/manager_node.cpp     # rclcpp 모드 발행 + 건강 모니터링
```

## 모드 상태 기계 (6상태)

```
IDLE ──(가압 명령)──▶ PRESSURIZING ──(에어베어링 안정)──▶ READY
                                                           │
                                               (GNC 시작 명령)
                                                           ▼
EMERGENCY_STOP ◀──(비상)── SAFE_HOLD ◀──(이상 감지)── ACTIVE
                                                     (GNC 실행)
```

### 상태 값

| 상태 | 값 | 설명 |
|------|-----|------|
| `IDLE` | 0 | 초기 상태, 모든 밸브 off |
| `PRESSURIZING` | 1 | 에어베어링 가압 중 |
| `READY` | 2 | 에어베어링 활성, GNC 대기 |
| `ACTIVE` | 3 | GNC 루프 실행 중 |
| `SAFE_HOLD` | 4 | 안전 정지 (장애 응답) |
| `EMERGENCY_STOP` | 5 | 비상 정지 (모든 출력 차단) |

### 전환 규칙

| 현재 상태 | 허용 전환 |
|-----------|-----------|
| IDLE | → PRESSURIZING, EMERGENCY_STOP |
| PRESSURIZING | → READY, IDLE, EMERGENCY_STOP |
| READY | → ACTIVE, IDLE, EMERGENCY_STOP |
| ACTIVE | → SAFE_HOLD, READY, EMERGENCY_STOP |
| SAFE_HOLD | → READY, IDLE, EMERGENCY_STOP |
| EMERGENCY_STOP | → IDLE (수동 리셋만 가능) |

- `emergency_stop()`: 어떤 상태에서든 즉시 EMERGENCY_STOP으로 전환
- 모든 전환은 `can_transition()` 검증 후 실행
- 전환 로그 유지: `list[(from_mode, to_mode, reason)]`

## 건강 감시 (HealthStatus)

| 항목 | 감시 방법 | 경고 | 장애 |
|------|-----------|------|------|
| 항법 데이터 나이 | `navigation_age_ms` (미수신 시 큰 값) | manager 한계의 1/2 초과 (기본 500ms) | manager 한계 초과 (기본 1000ms) |
| 제어 루프 주파수 | `control_loop_rate_hz` | < 15Hz | — |
| Modbus RTT | `modbus_rtt_ms` | > 50ms | — |
| 항법 불확실도 | EKF 공분산 1σ(`std_x/y/theta`) > 한계 | — | 초과 시 |
| 액추에이터 | `valve_status` 신선도(< 500ms) | — | stale 시 `actuators_ok=false`·arming 제한 (자동 SAFE_HOLD 조건에는 미포함) |

Vicon 상실 판정은 발행 주파수가 아니라 **EKF 공분산**으로 한다 — EKF는 Vicon
상실 후에도 추측항법으로 발행을 유지하므로 rate로는 감지 불가(불확실도만 증가).

### 자동 SAFE_HOLD 및 모드 게이팅

- 조건: `auto_safe_hold=True` + `mode==ACTIVE` + 장애 → ACTIVE → SAFE_HOLD 자동 전환
- **모드가 실제 출력을 차단**: `valve_driver_node`가 `/system/mode`를 구독해
  ACTIVE에서만 추력 허용, EMERGENCY_STOP 시 HAL `emergency_stop()`(추력0+메인밸브)

GNC의 `state_age_limit_ms`(기본 100ms)는 짧은 항법 미수신 동안 자체적으로 0 출력을 내는
가드다. manager의 별도 `nav_age_limit_ms`(기본 1000ms)는 지속적인 미수신을 장애로 판단해
SAFE_HOLD를 유지하는 운용 정책이다. 짧은 미수신이 회복되면 모드 변경 없이 제어를 재개할
수 있다. 불확실도 한계 초과는 이 시간과 별도로 장애로 판단한다. SAFE_HOLD에 진입한 뒤에는
데이터가 회복되어도 자동으로 ACTIVE로 복귀하지 않으며 준비도 충족 후 READY → ACTIVE
서비스 요청이 필요하다.

감시 주기도 `gnc_node.control_rate_hz`를 공유하므로 기본 20Hz(명목 50ms 주기)다.
두 타이머는 독립적으로 실행되며 주기가 같아도 실행 시점 동기화나 50ms 응답을 보장하지 않는다.
두 시간 한계는 동역학에서 도출한 안전 보장값이 아닌 임시 운용 설정이다. 변경하려면 실제
지연·지터·연속 누락과 플랜트 안전 여유를 측정해야 한다. EKF의 `vicon_timeout_ms`는
Vicon 보정 공백 감시를 위한 별도 정책으로 이 값과 결합하지 않는다.

### arming 전환 준비도 게이팅

`ManagerCore::set_mode`는 서브시스템 준비도가 충족될 때만 arming 전환을 허용한다(미충족 시
`success=False` + 사유). 안전 방향 전환(IDLE/SAFE_HOLD/EMERGENCY_STOP)은 절대 게이팅하지 않는다.
준비도는 통신·헬스 레벨(토픽 신선도 + EKF 수렴)로만 판정한다 — 압력센서 등 능동 자가진단 하드웨어는 없다.

| 대상 전환 | 준비 조건 |
|-----------|-----------|
| → PRESSURIZING | 액추에이터(`valve_status`) 신선 |
| → READY | 액추에이터 신선 + 메인밸브 개방 |
| → ACTIVE | 항법 수렴(`nav_ready`) + Vicon 신선 + 메인밸브 개방 |

`nav_ready` = `/navigation/state` 신선 + `navigation_ok` + `!navigation_degraded`.
액추에이터 신선도는 `valve_driver`가 heartbeat마다 `valve_status`를 재발행하므로 ARM 전에도 관측된다.

## ROS2 인터페이스

| 유형 | 이름 | 타입 | 설명 |
|------|------|------|------|
| 구독 | `/navigation/state` | `SpacecraftState` | 항법 상태 (나이 추적) |
| 구독 | `/sensors/vicon_state` | `SpacecraftState` | Vicon 신선도 (위치 소스 생존) |
| 구독 | `/actuators/valve_status` | `ValveStatus` | 액추에이터 상태 |
| 발행 | `/system/mode` | `SystemMode` | 건강 점검 주기(기본 20Hz) 및 서비스 전환 직후 |
| 발행 | `/system/health` | `SystemHealth` | 건강 점검 주기(기본 20Hz) |
| 서비스 | `/system/set_mode` | `SetSystemMode` | 모드 변경 요청 |

## 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `health_check_rate_hz` | `${gnc_node.control_rate_hz}` (20.0) | GNC와 같은 명목 감시 주기 |
| `nav_age_limit_ms` | 1000.0 | 지속적인 항법 미수신을 SAFE_HOLD로 래치하는 한계 |
| `control_rate_min_hz` | 15.0 | 제어 루프 최소 주파수 |
| `auto_safe_hold` | `True` | ACTIVE 모드 장애 시 자동 SAFE_HOLD |
| `nav_pos_std_limit` | 0.005 | 위치 1σ 한계 [m] (초과 시 항법 저하) |
| `nav_heading_std_limit` | 0.02 | 자세 1σ 한계 [rad] |

## 테스트

`tests/cpp_manager_test.cpp`와 `tests/cpp_manager_ros_test.cpp`에서 실제 C++ 구현을 검사한다.

- 36개 FSM 전이 조합, E-STOP·수동 리셋과 arming 준비도
- 1000ms 경계, 짧은 미수신 회복, 지속 장애 SAFE_HOLD·수동 재가동
- 실제 ROS 토픽·서비스, 불확실도 장애, 서비스 전환 직후 모드 발행

`tests/arm/test_system_integration.py`는 컴파일된 manager·SimHal 밸브 프로세스와
dry-run 로봇팔 사이의 서비스 순서, ROS 상태 피드백과 E-STOP을 검사한다.

## 향후 작업

- PLC heartbeat 타임아웃 → EMERGENCY_STOP (래더 측 구현 필요)
- 건강 임계값 개별 파라미터화 (현재 Modbus RTT 50ms 고정)
