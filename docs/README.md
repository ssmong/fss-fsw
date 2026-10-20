# FSS FSW 문서 가이드

패키지별 상세 문서 인덱스. **`src/`와 동일한 카테고리 트리**(infra/hardware/gnc/ops)로 분류한다 —
`src/<카테고리>/<패키지>` 를 찾으면 문서는 `docs/<카테고리>/` 에 있다.

## infra/ — 기반 인프라 (`src/infra/`)

| 문서 | 패키지 | 상태 | 내용 |
|------|--------|:----:|------|
| [001_Interfaces](infra/001_Interfaces.md) | fss_interfaces | ✅ | ROS2 커스텀 msg 6종 + srv 4종 정의 |
| [002_Common](infra/002_Common.md) | fss_common | ✅ | 경로 탐색, Hydra→ROS2 브릿지, 런치 유틸, 수학 |
| [011_Bringup](infra/011_Bringup.md) | fss_bringup | ✅ | 런치 파일 7개, 팔·추력기 독립 선택 옵션 |
| [012_Configuration](infra/012_Configuration.md) | config/ | ✅ | Hydra(실험 전 설정) + ROS2 params(런타임) 2단계 |

## hardware/ — 하드웨어 계층 (`src/hardware/`)

| 문서 | 패키지 | 상태 | 내용 |
|------|--------|:----:|------|
| [003_HAL](hardware/003_HAL.md) | fss_hal | ✅ | PLC·IMU·Vicon·반응휠의 장치별 통신·변환과 공통 계약 |
| [004_Actuators](hardware/004_Actuators.md) | fss_actuators | ✅ | 밸브 드라이버, 반응휠 USB/모의 드라이버 |
| [005_Sensors](hardware/005_Sensors.md) | fss_sensors | ✅ | FSS_VICON UDP·Xsens IMU HAL과 ROS 브릿지 |
| [006_Arm](hardware/006_Arm.md) | fss_arm | 모의 검증 | 위치·속도·전류 제어, 하드웨어 매핑, 상태·주기 감시 |
| [007_Arm_Commands](hardware/007_Arm_Commands.md) | fss_arm / fss_arm_msgs | 모의 검증 | 위치·속도·전류 토픽, 모드 전환 서비스, 시작·정지 예제 |
| [014_Mass_Properties](hardware/014_Mass_Properties.md) | 본체·로봇팔·그리퍼 | 측정/추정 기록 | +X 좌표계, 질량·COM·관성 계산식과 다이어그램 |

## gnc/ — GNC 파이프라인 (`src/gnc/`)

| 문서 | 패키지 | 상태 | 내용 |
|------|--------|:----:|------|
| [006_Navigation](gnc/006_Navigation.md) | fss_navigation | ✅ | 11-state EKF (관성 프레임, IMU strapdown + Vicon 융합 + 바이어스 추정) |
| [007_GNC](gnc/007_GNC.md) | fss_gnc | ✅ | 유도(target/trajectory) + 제어(PID/LQR) + QP 할당 |
| [008_Calibration](gnc/008_Calibration.md) | fss_tuning | ⏳ | 밸브 캘리브레이션 (데드밴드, 히스테리시스, 추력 매핑) |

## ops/ — 시스템 운영 (`src/ops/`)

| 문서 | 패키지 | 상태 | 내용 |
|------|--------|:----:|------|
| [009_Manager](ops/009_Manager.md) | fss_manager | ✅ | 6상태 FSM (IDLE→ACTIVE→SAFE_HOLD) + 건강 감시 |
| [010_Simulation](ops/010_Simulation.md) | fss_sim | ✅ | SIL 플랜트 모델 (3-DOF + PVQ 카탈로그 곡선 + 센서 노이즈) |
| [013_Arm_Execution](ops/013_Arm_Execution.md) | 본체·팔 통합 | 부분 검증 | 장치 선택, 기동·모드 전환·팔 명령·종료 절차 |

## 참조

| 문서 | 내용 |
|------|------|
| [../README.md](../README.md) | 프로젝트 개요, 패키지 상태, 빌드·실행 |

## 자료 (Markdown 외)

| 경로 | 내용 |
|------|------|
| `hardware/Diagram_pneumatic_system.jpg` | 공압 계통도 (탱크→레귤레이터→추력/부유 라인) |
| `hardware/Diagram_electric_system.png` | 전장 계통도 |
