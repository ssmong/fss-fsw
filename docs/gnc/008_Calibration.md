# 008. fss_tuning — 밸브 캘리브레이션

**상태: 실측 정적 추력 맵 지원 완료 · 자동 식별은 Phase 5**
**패키지 유형:** `ament_python`

## 목적

비례밸브 8개의 데드밴드, 선형성, 스텝 응답 특성을 측정하고
보상 모델을 생성하여 드라이버 역변환·추정 추력, SIL에 적용한다.

## 대상 특성

**SMC PVQ33-5G-16-01N-X33** (전류 드라이버 내장 특주, φ1.6mm)

| 항목 | 값 |
|------|-----|
| 히스테리시스 | 개방/폐쇄 경로 불일치 (데이터시트 유량 곡선) |
| 개방 시작 | 밸브별 실측 대상 — 현재 드라이버 임계값 차단은 비활성(0) |
| 전류-추력 | 비선형 (오리피스 유량 특성) |
| 최대 압력차 | 0.7 MPa — 공급 80psi(0.55 MPa)는 정격 내 |
| 추정 추력 | 0.6–1.5 N/노즐 (Phase 5 Vicon 변위 실측) |
| 스텝 응답 | 밸브별 편차 가능 |

## 계획 구조

```
fss_tuning/
├── __init__.py
├── deadband_identifier.py      # 데드밴드 임계값 식별
├── linearity_mapper.py         # 입력-추력 비선형 매핑
├── step_response_node.py       # 스텝 응답 측정 노드
└── calibration_model.py        # 보상 모델 생성 및 저장
```

## 계획된 캘리브레이션 절차

### 0. 밸브 입구 압력 확인

캘리브레이션 전에 **밸브 입구 압력을 압력 게이지로 실측**한다.
다이어그램의 2차 레귤레이터는 실물에서 미확인 (2026-04-01 점검).

### 1. 데드밴드 식별

각 밸브에 대해 0에서 서서히 값을 올리며 실제 움직임이 시작되는 지점을 기록.
Vicon으로 미세 변위를 감지하여 자동 측정.
PVQ33 데이터시트: ~8mA(PLC≈1000)에서 개방 시작. 실측이 크게 다르면 배관 저항 의심.

```
결과 → calibration/deadband_values.yaml
  valve_0_deadband: 1980
  valve_1_deadband: 2050
  ...
```

### 2. 선형성 매핑

데드밴드 이상 영역에서 여러 입력값에 대한 실측 추력을 측정.
`F = f(input)` 곡선을 생성하여 비선형 보상에 사용.

### 3. 히스테리시스 측정

각 밸브에 대해 **상승/하강 양방향 스윕**을 수행하여 개폐 경로 차이를 기록.
- PLC값 0→4000 (상승) + 4000→0 (하강) 스윕
- 각 지점에서 Vicon 변위로 실제 추력 측정
- 결과: `calibration/hysteresis_curves.yaml` (밸브별 상승/하강 룩업 테이블)
- `valve_driver_node`에서 현재 방향(증가/감소)에 따라 역함수 적용

### 4. 스텝 응답 측정

밸브 명령 → 실제 추력 발생까지의 시간 지연 및 상승 시간을 측정.
SIL 밸브 모델(τ_plc, τ_valve 파라미터) 튜닝에 활용.

## 출력 파일

| 파일 | 위치 | 용도 |
|------|------|------|
| `deadband_values.yaml` | `calibration/` | 밸브별 데드밴드 임계값 |
| `thrust_curves.yaml` | `calibration/` | 입력-추력 매핑 곡선 |
| `step_response.yaml` | `calibration/` | 밸브별 응답 시간 특성 |
| `hysteresis_curves.yaml` | `calibration/` | 밸브별 상승/하강 유량 곡선 |

## 현재 정적 맵 지원

`fss_common.valve_thrust_map`이 밸브별 구간 선형 보간과 역보간을 제공한다.
`config/calibration/valve_thrust_measured.yaml`은 실측 기반 정적 맵이다.
`duty`/`thrust_n` 형식으로 저장하며 `valve_thrust_map_file`에서 선택한다.
GNC는 맵의 최대 추력으로 할당해 목표 추력[N]을 발행한다. 드라이버는 이를 밸브 입력으로
역변환하고, 적용 입력의 정적 추정 추력도 발행한다.
SIL의 `mapped` 모드도 같은 파일을 사용한다. [실행 방법](../ops/010_Simulation.md#밸브별-입력추력-맵).

## 의존성

- `fss_hal` (밸브 명령 전송)
- `fss_sensors` (Vicon으로 변위 측정)
- `matplotlib` (결과 시각화)
- `numpy`, `scipy` (곡선 피팅)

## 선행 요구사항

- Phase 2 하드웨어 연동 완료 (PLC 통신 정상)
- Phase 3 센서 (Vicon 포즈 데이터 수신)
