# 006. fss_navigation — 항법 (상태 추정)

**상태: 완료 (11-state EKF, 2026-06-21 관성 프레임 + 테이블 바이어스 재설계)**
**패키지 유형:** `ament_cmake` (C++17, Eigen)

## 개요

11-state 확장 칼만 필터(EKF)로 Vicon과 IMU 데이터를 융합하여
우주선 상태 + 센서 바이어스를 실시간 추정한다.
IMU strapdown integration 방식으로 IMU가 predict를 구동하고,
Vicon이 measurement update를 수행한다.

참고 문헌:
- Crassidis & Junkins, "Optimal Estimation of Dynamic Systems", 2nd Ed, Ch. 7
- Groves, "Principles of GNSS, Inertial, and Multisensor Navigation", 2nd Ed, Ch. 14

## 모듈 구조

```
fss_navigation/
├── include/fss_navigation/  # EKF·항법 코어 인터페이스
├── src/ekf.cpp              # 11-state EKF (ROS2 독립)
├── src/navigation.cpp       # 시각·readiness 처리
├── src/ekf_node.cpp         # ROS2 구독·발행·타이머
└── test/                   # native/ROS 검사와 고정 golden fixture
```

중복 Python EKF·노드는 제거했다. 이전 구현에서 동결한 741회 연산의 상태·공분산·NIS·게이팅
결과를 `test/ekf_golden.npz`에 보존하며, 출처와 대체 검사 목록은 `test/README.md`에 기록한다.

## EKF 설계

### 상태 벡터 (11×1)

```
x = [px, py, θ, vx, vy, ω, b_g, b_ax, b_ay, b_tx, b_ty]ᵀ
```
| 인덱스 | 상태 | 단위 | 프레임 | 설명 |
|--------|------|------|--------|------|
| 0, 1 | px, py | m | 테이블 | 위치 |
| 2 | θ | rad | — | 방향각 |
| 3, 4 | vx, vy | m/s | **테이블** | 속도 (관성 프레임) |
| 5 | ω | rad/s | 본체 | 바이어스 보정된 각속도 |
| 6 | b_g | rad/s | 본체 | 자이로 Z축 바이어스 |
| 7, 8 | b_ax, b_ay | m/s² | 본체 | 가속도계 바이어스 (MEMS 센서) |
| 9, 10 | b_tx, b_ty | m/s² | **테이블** | 가속도 바이어스 (틸트 중력 g·sinα 등) |

- 속도는 테이블(관성) 프레임 — 병진 동역학에 transport 항(−ω×v)이 없다.
  관성→본체 회전은 컨트롤러 출력단(`gnc_node`)에서 1회 수행.
- 가속도 바이어스는 본체(b_ax,b_ay) / 테이블(b_tx,b_ty) 이중 모델.
  본체 바이어스는 R(θ) 회전 전 차감, 테이블 바이어스는 회전 후 가산.
- **IMU 입력 계약**: `/sensors/imu`는 `imu_bridge_node`가 `body` 프레임으로 변환한 값.
  EKF가 `gyro_z`, `accel_x`, `accel_y`를 yaw로 회전시키므로, 브리지 앞에서
  table 프레임으로 미리 projection하면 회전이 이중 적용된다.

### 프로세스 모델 (IMU strapdown, 관성 프레임)

```
ṗx  = vx                                 (관성 프레임 속도 = 위치 미분)
ṗy  = vy
θ̇   = ω_imu − b_g                        (바이어스 보정 자이로)
v̇x  = (ax_imu − b_ax)·cos θ − (ay_imu − b_ay)·sin θ + b_tx   (본체 비력 회전 + 테이블 바이어스)
v̇y  = (ax_imu − b_ax)·sin θ + (ay_imu − b_ay)·cos θ + b_ty
ω    = ω_imu − b_g                       (직접 대입)
ḃ_g  = 0 + w_bg                          (랜덤 워크)
ḃ_ax = 0 + w_bax,  ḃ_ay = 0 + w_bay
ḃ_tx = 0 + w_bt,   ḃ_ty = 0 + w_bt       (준정적, 매우 작은 RW)
```

- **적분**: strapdown 표준 — θ는 닫힌 해, 속도는 중간점 θ로 비력 회전, 위치는 사다리꼴.
- **야코비안**: 1차 이산 야코비안 F (11×11), 바이어스→상태 커플링 포함:
  - `F[px, vx] = dt`, `F[py, vy] = dt` (관성 속도 → 위치, 회전 없음)
  - `F[θ, b_g] = -dt` (자이로 바이어스 → heading drift)
  - `F[vx, θ] = (-ax_c·sinθ - ay_c·cosθ)·dt`, `F[vy, θ] = (ax_c·cosθ - ay_c·sinθ)·dt` (비력 회전의 θ 의존)
  - `F[vx, b_ax] = -cosθ·dt`, `F[vx, b_ay] = sinθ·dt` (본체 바이어스 → 관성 속도, R(θ) 회전)
  - `F[vy, b_ax] = -sinθ·dt`, `F[vy, b_ay] = -cosθ·dt`
  - `F[vx, b_tx] = dt`, `F[vy, b_ty] = dt` (테이블 바이어스 → 관성 속도, 회전 없음)
  - `F[ω, b_g] = -1` (직접 대입 의존)

### 프로세스 노이즈 Q (데이터시트 기반)

연속시간 스펙트럼 밀도 → 이산 변환: `Q_d ≈ Q_c · dt`

| 상태 | Q 공식 | 센서 사양 |
|------|--------|-----------|
| px, py | `1e-10 · dt` | 적분 드리프트 (매우 작음) |
| θ | `σ²_arw · dt` | MTi-630 ARW: 0.007°/s/√Hz |
| vx, vy | `σ²_vrw · dt` | MTi-630 VRW: 60μg/√Hz |
| ω | `σ²_arw` | 자이로 직접 측정 (dt 미적용) |
| b_g | `σ²_bg_rw · dt` | bias random walk ~1e-5 rad/s/√s |
| b_ax, b_ay | `σ²_ba_rw · dt` | 본체 bias random walk ~1e-4 m/s²/√s |
| b_tx, b_ty | `σ²_bt_rw · dt` | 테이블 bias random walk ~1e-5 m/s²/√s (준정적) |

### 측정 모델 (Vicon)

Vicon은 [px, py, θ]를 직접 관측한다 (선형 측정 행렬 H, 3×11).
IMU는 process input으로만 사용 (measurement update 아님).

**이상치 게이팅**: 마할라노비스 거리 기반 χ² 검정 (자유도 3, 99% 신뢰구간)
```
NIS = yᵀ S⁻¹ y
if NIS > 11.345:  # χ²₃(0.99)
    측정 기각
```

**NIS 모니터링**: Normalized Innovation Squared 이동 평균으로 필터 일관성 진단.
기대값 ≈ 3.0. 범위 [0.5, 8.0] 이탈 시 Q/R 튜닝 필요 경고.

### 바이어스 관측가능성

Vicon이 [px, py, θ]만 관측하지만, F 행렬의 커플링을 통해 바이어스가 관측 가능:
- `b_g → θ` (heading 적분) → Vicon θ 관측으로 수렴
- `b_ax → vx → px` (속도→위치 적분) → Vicon px 관측으로 수렴
- `b_ay → vy → py` (동일 경로)
- `b_tx, b_ty → vx, vy → px, py` (테이블 바이어스도 속도→위치 경로로 관측)

본체/테이블 가속도 바이어스는 **θ가 변하는 회전 기동에서만 분리 관측**된다
(정지유지 중에는 둘의 합만 관측). 검증: `test_accel_bias_converges`,
`test_table_tilt_bias_converges`.

Phase 5에서 실측 IMU/Vicon으로 NIS 기반 Q/R 재튜닝 필요.

### Vicon 장애 대응

1. `vicon_timeout_ms` (기본 100ms) 초과 → prediction-only 모드 (IMU strapdown dead reckoning)
2. `max_prediction_cycles` (기본 50) 초과 → 경고 로그 (dead reckoning 지속 알림)
3. 상태 발행에 공분산 1σ(`std_x/std_y/std_theta`) 포함 — 추측항법 중 증가하며,
   매니저가 `nav_pos_std_limit`/`nav_heading_std_limit` 초과 시 항법 저하로 판정(자동 SAFE_HOLD)
4. Vicon 복구 시 자동 정상 모드 복귀 (불확실도도 수 샘플 내 수렴)
5. 바이어스 추정 덕분에 dead reckoning 정확도 향상 (비보정 대비 ~10배)

### 공분산 갱신

Joseph form (수치 안정성, 양정부호 보장):
```
P = (I − KH) P (I − KH)ᵀ + K R Kᵀ
P = 0.5(P + Pᵀ)  # 대칭 보장
```

## ekf_node ROS2 인터페이스

| 유형 | 이름 | 타입 | 설명 |
|------|------|------|------|
| 구독 | `/sensors/vicon_state` | `SpacecraftState` | Vicon 위치/자세 |
| 구독 | `/sensors/imu` | `sensor_msgs/Imu` | IMU 각속도/가속도 |
| 발행 | `/navigation/state` | `SpacecraftState` | 추정 상태 (50Hz) |

### 파라미터 (ekf.yaml, 단일 설정 소스)

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `gyro_noise_density` | `1.2217e-4` | ARW [rad/s/√Hz] (MTi-630: 0.007°/s/√Hz) |
| `accel_noise_density` | `5.886e-4` | VRW [m/s²/√Hz] (MTi-630: 60μg/√Hz) |
| `gyro_bias_random_walk` | `1e-5` | 자이로 bias RW [rad/s/√s] |
| `accel_bias_random_walk` | `1e-4` | 가속도 bias RW [m/s²/√s] |
| `vicon_position_noise` | `5e-4` | Vicon 위치 σ [m] (MX T10: 0.5mm) |
| `vicon_heading_noise` | `1.745e-3` | Vicon 방향 σ [rad] (~0.1°) |
| `gate_threshold` | `11.345` | 이상치 게이팅 (χ²₃ 0.99) |
| `nis_window` | `50` | NIS 이동 평균 윈도우 |
| `publish_rate_hz` | `50.0` | 상태 발행 주파수 |
| `vicon_timeout_ms` | `100.0` | Vicon 타임아웃 [ms] |
| `max_prediction_cycles` | `50` | prediction-only 최대 주기 |

## 실시간 모니터링 대시보드

`scripts/live_plot.py`는 PySide6 + pyqtgraph 기반 미션 대시보드다.
노트북에서 실행하여 Jetson의 ROS2 토픽을 실시간 표시하며, OpenGL 렌더링은 선택 사항이다.

```bash
# 노트북에서 (같은 WiFi, 같은 ROS_DOMAIN_ID)
pip3 install PySide6 pyqtgraph PyOpenGL   # 최초 1회
python3 scripts/live_plot.py
python3 scripts/live_plot.py --opengl     # 선택 사항
```

초기 화면에는 모드, 항법, 액추에이터, 메인 밸브, XY 궤적, 현재/목표/오차만 표시한다.
상태 시계열, EKF 공분산, 명령/적용 밸브 값, 시스템 로그는 하단 탭에서 확인한다.
새 시계열은 `Dashboard._build_detail_tabs()`에 `SeriesSpec`을 추가하여 공통 스타일과 갱신 로직을 재사용한다.

## 실행

```bash
# 항법만 실행
ros2 launch fss_bringup navigation.launch.py

# 전체 시스템
ros2 launch fss_bringup fss_full.launch.py
```

## 설정 파일

- `config/gnc/navigator/ekf.yaml` — EKF 전체 파라미터 (단일 설정 소스)
- `navigation.launch.py`에서 인라인 파라미터 사용하지 않음 (YAML 직접 로드)
