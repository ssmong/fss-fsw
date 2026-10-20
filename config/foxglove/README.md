# Foxglove 레이아웃 가이드

Foxglove는 원격 모니터링과 자유로운 plot 추가에 사용하고, `scripts/live_plot.py`는 실험 중 핵심 상태를 빠르게 판단하는 고정 관제 화면으로 사용합니다.

## 레이아웃 선택

| 파일 | 용도 | 초기 화면 |
| --- | --- | --- |
| `hardware_monitor.json` | 실제 하드웨어 및 일반 운용 | 모드, 항법, 액추에이터, 메인 밸브, 추종 상태 |
| `sil_ekf_tuning.json` | SIL 폐루프와 EKF 튜닝 | 참값/추정 비교, 추정·추종 오차, 공분산, 밸브 |
| `imu_debug.json` | IMU 장착 축과 브리지 검증 | Xsens raw/FSS body 비교와 평면 yaw |

기본 운용 화면에는 즉시 판단에 필요한 정보만 둡니다. 카메라, 전체 ROS 로그, 세부 센서 파형처럼 특정 작업에서만 필요한 패널은 별도 레이아웃에서 확인합니다.

## 연결 및 불러오기

```bash
# Jetson
bash scripts/launch/start_foxglove.sh

# 노트북
ssh -L 8765:localhost:8765 <user>@<jetson-ip>
```

Foxglove에서 `ws://localhost:8765`에 연결한 뒤 `Layouts → Import from file...`에서 필요한 JSON 파일을 선택합니다.

## Plot 추가 방법

1. 왼쪽 `Topics`에서 숫자 필드를 찾습니다.
2. 필드를 기존 Plot으로 끌어 넣거나 새 Plot 패널로 끌어 놓습니다.
3. 센서 데이터는 가능하면 timestamp를 `Header stamp`로 선택합니다.
4. 제목과 단위를 명시하고, 한 패널에는 같은 단위의 값만 묶습니다.
5. 패널을 기존 layout에 저장한 뒤 JSON으로 다시 export합니다.

여러 토픽을 계산해야 하는 파생값은 Foxglove User Script로 만듭니다. `sil_ekf_tuning.json`의 `FSS Derived Metrics`가 참값, EKF, 기준 상태를 결합해 오차를 만드는 예제입니다.

## 레이아웃 원칙

- 초록은 정상, 노랑은 주의, 빨강은 즉시 확인이 필요한 상태로 사용합니다.
- 상태가 아닌 데이터 선에서는 초록/빨강의 의미를 사용하지 않습니다.
- Vicon은 파란 점선·점, EKF는 청록 실선, 기준값은 노랑 파선을 기본으로 합니다.
- 색상만으로 선을 구분하지 않고 범례 이름과 선 굵기를 함께 유지합니다.
- `hardware_monitor`에는 운용 결정을 바꾸는 정보만 추가합니다.
- 원시 센서 파형과 튜닝용 plot은 별도 레이아웃에 추가합니다.
- 실제 차이를 계산하지 않은 값을 “오차”로 표시하지 않습니다.
- 같은 의미의 축과 선 색상은 레이아웃 사이에서도 유지합니다.
