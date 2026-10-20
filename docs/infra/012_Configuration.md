# 012. 설정 시스템 (Hydra SSoT)

**상태: 완료** (2026-07-14 전면 재구조화)

## 개요

유일한 설정 소스는 리포 루트의 **`config/` Hydra 트리**다.
런치 파일이 실행 시점에 이 트리를 컴포즈해 노드별 ROS2 파라미터 YAML을 생성·로드한다.

```
config/ (Hydra, SSoT)
   │  fss_common.hydra_bridge.generate_node_param_files()
   ▼
/tmp/fss_params_*/<노드명>.yaml  (노드명: ros__parameters: ...)
   │  런치: parameters=[node_params("<노드명>")]
   ▼
노드: declare_parameter(이름, 타입)  ← rclcpp/rclpy에서 타입만 선언, 기본값 없음
```

핵심 규칙:
- **노드는 기본값을 갖지 않는다** — YAML이 값을 안 주면 `ParameterUninitializedException`으로
  기동 즉시 실패. 값의 출처가 항상 config 트리임이 보장된다.
  (예외: `use_sim`, `use_sim_time` 등 런치 주입 실행 모드 플래그)
- **런치 인라인 파라미터 dict 금지** — 전 런치가 `node_params()` 사용.
- **공유 값은 보간으로 참조** — 권위 값은 한 곳에만 존재.

## 트리 구조

각 파일 상단의 `# @package <노드명>` 지시자가 내용을 해당 노드의 파라미터
서브트리로 병합한다. 중첩 키는 ROS2 로드 시 점(.)으로 평탄화된다
(`translational: {kp: ...}` → `translational.kp`).

```
config/
├── config.yaml                  # defaults 목록 (_self_ 뒤 experiment가 실행 플래그도 override)
├── vehicle/mass_properties.yaml # @package vehicle — 공유 물리 권위 값
│                                #   mass, inertia_zz, max_thrust_per_nozzle,
│                                #   max_force_body(2N), max_torque_body(0.2N·m)
├── vehicle/thruster_geometry.yaml  # 데이터 파일 (Hydra 외부, 노드가 config_file()로 직접 로드)
├── gnc/node.yaml                # @package gnc_node — 공통 (rate, PID/LQR 게인)
├── gnc/controller/{pid,lqr}.yaml  # @package gnc_node — type 선택만
├── gnc/controller/lqr_gains.yaml      # K 데이터 (compute_lqr_gains.py 생성, Hydra 외부)
├── gnc/guidance/{station_keep,trajectory}.yaml  # @package gnc_node
├── gnc/navigator/ekf.yaml       # @package ekf_node
├── gnc/allocator/thruster_allocation.yaml  # @package gnc_node
├── hardware/plc.yaml            # @package valve_driver_node
├── hardware/imu.yaml            # @package imu_bridge_node
├── hardware/vicon.yaml          # @package vicon_bridge_node
├── hardware/arm.yaml            # @package arm_driver_node — 관절 명령·Dynamixel 설정
├── hardware/realsense.yaml      # realsense2_camera 4.x flat config (Hydra 외부)
├── hardware/reaction_wheel.yaml # SAMD21 MCU 계약·host timeout (독립 반응휠 노드가 직접 로드)
├── system/manager.yaml          # @package manager_node
├── system/sim.yaml              # @package plant_model_node (노이즈는 ${ekf_node.*} 보간)
├── system/sim_valve/ideal.yaml  # 지연·비선형성이 없는 SIL 밸브
├── system/sim_valve/real.yaml   # X33 축척·압력 조건을 지정하는 현실 모델 (기본)
├── system/sim_valve/pvq33_5g_16_x33_flow_curve.yaml  # 압력별 상승·하강 LUT
└── experiment/*.yaml            # @package _global_ — 그룹 override + 실험 메타데이터
```

## 보간(공유 값) 흐름

| 권위 값 | 소비처 |
|---------|--------|
| `vehicle.mass`, `vehicle.inertia_zz` | `plant_model_node.*` (SIL 동역학) |
| `vehicle.max_thrust_per_nozzle` | `gnc_node.allocator.*`(B 스케일), `plant_model_node.max_thrust_per_valve` |
| `vehicle.max_force_body/max_torque_body` | 전 제어기 한계 (기하 유도: 2N/0.2N·m, Phase 5 실측 후 갱신) |
| `ekf_node.*_noise*` | `plant_model_node.*` (SIL 노이즈 = EKF 가정과 자동 일치) |
| `gnc_node.control_rate_hz` | `manager_node.health_check_rate_hz` (명목 감시 주기) |

## 실행 방법

```bash
# 기본 (런치가 직접 컴포즈)
ros2 launch fss_bringup fss_full.launch.py

# 오버라이드 (환경변수)
FSS_HYDRA_OVERRIDES="gnc/controller=lqr gnc_node.translational.kp=8.0" \
  ros2 launch fss_bringup sim.launch.py

# 실험 러너 (오버라이드 자동 전달 + 컴포즈 결과 출력)
python scripts/run_experiment.py experiment=trajectory_tracking
python scripts/run_experiment.py gnc/controller=lqr vehicle.max_thrust_per_nozzle=1.2
python scripts/run_experiment.py system.use_sim=true system/sim_valve=ideal
python scripts/run_experiment.py system.use_sim=true system.use_arm=true system.use_thrusters=false
```

러너는 `config.yaml`의 `system.use_sim/use_arm/use_thrusters/use_gnc`를 읽어 런치 인자로
전달한다. 직접 `ros2 launch`로 실행하면 `use_arm:=true use_thrusters:=false`처럼 지정한다.
팔의 모의/실장치 연결은 본체 `use_sim`과 독립적인 `arm_driver_node.dry_run` 설정이다.
`experiment=thrust_characterization`은 외부 밸브 명령과 충돌하지 않도록 `system.use_gnc=false`를
적용한다. 명시한 CLI 오버라이드는 프리셋보다 우선한다.

러너는 설정을 적용해 시스템을 실행하며 종료는 Ctrl+C로 한다. `experiment.duration_sec`와
`record_bag`는 메타데이터로 유지하지만, 현재 자동 종료·rosbag 자동 기록은 구현하지 않는다.
기록이 필요하면 별도 터미널에서 rosbag을 수동으로 시작·종료한다. 게인을 비교할 때도
각 실행을 직접 종료한 뒤 다음 값을 적용한다.

## 정합성 검증

노드 `declare_parameter` 목록과 config 생성 키의 1:1을 검증한다.
**노드 파라미터를 추가/삭제하면 이 스크립트의 EXPECTED도 함께 갱신할 것.**

```bash
python scripts/validate_config.py
# 기본·제어기·유도 조합 및 5개 실험 프리셋 × 8개 노드
# + 팔 ACTIVE 만료 시간과 manager 발행 주기의 정합성
# + 독립적인 GNC/manager 항법 미수신 한계와 명목 감시 주기 보간
# + realsense/reaction_wheel 외부 하드웨어 YAML 계약 검사
```

`realsense.yaml`(외부 드라이버가 직접 읽음)과 `reaction_wheel.yaml`(독립 반응휠 노드가 직접 읽음)은
Hydra defaults 밖이며, 검증 스크립트가 각각의 계약을 별도로 확인한다.

GNC의 `state_age_limit_ms`(기본 100ms)는 짧은 항법 미수신 시 자체 0 출력 가드이며,
manager의 `nav_age_limit_ms`(기본 1000ms)는 지속 장애 SAFE_HOLD를 위한 별도 한계다.
manager의 `health_check_rate_hz`는 `gnc_node.control_rate_hz`를 보간해 기본 20Hz로 감시한다.
각 그룹 YAML을 단독으로 읽지 않고
`load_hydra_config()` 또는 `generate_node_param_files()`로 전체 트리를 컴포즈·해석한다.
두 시간 한계는 임시 운용 설정이며 동역학 기반 안전 보장값이 아니다. 실측 지연·지터·연속
누락과 플랜트 안전 여유로 조정하고, 별도 센서 입력 정책인 EKF의 `vicon_timeout_ms`와 구분한다.

## 런타임 조정

실행 중 `ros2 param set /gnc_node translational.kp 3.0` 등으로 동적 조정 가능
(단, 노드가 콜백에서 재읽는 파라미터에 한함 — 대부분 기동 시 1회 읽음).
영구 변경은 반드시 config 트리에 반영한다.
