# 011. fss_bringup — 런치 및 실행

**상태: 완료**
**패키지 유형:** `ament_python`

## 개요

FSS 시스템의 런치 파일과 파라미터 YAML을 관리한다.
실제 하드웨어 모드와 SIL 시뮬레이션 모드를 `use_sim` 인자로 전환한다.

## 런치 파일 (7개)

### actuators.launch.py

밸브 드라이버 노드 단독 실행.

```bash
ros2 launch fss_bringup actuators.launch.py              # 실제 하드웨어
ros2 launch fss_bringup actuators.launch.py use_sim:=true # 시뮬레이션
ros2 launch fss_bringup actuators.launch.py use_sim:=true use_thrusters:=false # 추력 차단
```

### sensors.launch.py

센서 브릿지 노드 (Vicon, IMU, RealSense) 실행.

```bash
ros2 launch fss_bringup sensors.launch.py
ros2 launch fss_bringup sensors.launch.py use_imu:=false use_realsense:=false  # Vicon만
ros2 launch fss_bringup sensors.launch.py use_realsense:=true                 # D435i 포함
```

### navigation.launch.py

EKF 항법 노드 실행. 파라미터는 `config/gnc/navigator/ekf.yaml`에서 로드.

```bash
ros2 launch fss_bringup navigation.launch.py
```

### gnc.launch.py

GNC 노드 (유도 + 제어 + 할당) 실행. CPU 피닝, 선택적 SCHED_FIFO, 로그 레벨 지원.

```bash
ros2 launch fss_bringup gnc.launch.py
ros2 launch fss_bringup gnc.launch.py cpu_pin:=true cpu_core:=4 log_level:=debug
```

### fss_full.launch.py

전체 시스템 런치 (서브시스템 런치 파일 조합).

```bash
ros2 launch fss_bringup fss_full.launch.py                              # 실제 하드웨어
ros2 launch fss_bringup fss_full.launch.py use_sim:=true                # SIL 모드
ros2 launch fss_bringup fss_full.launch.py use_realsense:=true          # D435i 선택 실행
ros2 launch fss_bringup fss_full.launch.py cpu_pin:=true log_level:=debug  # CPU 피닝 + 디버그
ros2 launch fss_bringup fss_full.launch.py use_sim:=true use_arm:=true use_thrusters:=false
ros2 launch fss_bringup fss_full.launch.py use_sim:=true use_arm:=true use_thrusters:=true
```

포함 서브시스템:

- actuators (밸브 드라이버, CPU 코어 5 피닝 가능; use_sim 시 SimHal)
- sensors (Vicon + Xsens IMU 브릿지, D435i 선택 실행; 하드웨어 모드만)
- plant_model (use_sim 시에만 — 센서 토픽 + /clock 공급)
- navigation (11-state EKF, 관성 프레임)
- arm (`use_arm:=true`일 때 위치·속도·전류 명령 노드)
- gnc (`use_thrusters`와 `use_gnc`가 모두 true일 때; CPU 코어 4 피닝 가능)
- manager (모드 FSM + 건강 감시)

`use_thrusters:=false`이면 밸브 드라이버는 외부 명령을 받아도 추력 출력 8개를 0으로
전송하며, SIL 플랜트도 추력에 의한 힘·모멘트를 0으로 적용한다. 에어베어링 메인밸브,
PLC 연결, 밸브 상태, manager와 항법은 계속 실행한다. `use_arm`과 독립적으로 선택한다.

운용 절차: 기동 런치의 추력 허용과 관리자 모드 **ACTIVE**가 필요하다.
기동 직후는 IDLE — `/system/set_mode` 서비스로 IDLE→PRESSURIZING→READY→ACTIVE 전이 필요.
추력 ON 상태의 기존 모드 미수신·명령 만료 제한은 [통합 점검](../ops/013_Arm_Execution.md)을 따른다.

### arm.launch.py

팔 노드 단독 실행. 설정은 `config/hardware/arm.yaml`을 사용한다.
manager의 ACTIVE 신호와 `/arm/set_enabled`가 필요하다.

```bash
ros2 launch fss_bringup arm.launch.py
```

전체 런치에 `use_arm:=true`를 지정했다면 별도로 실행하지 않는다.

### sim.launch.py

SIL 모드 전용. `fss_full.launch.py`를 `use_sim:=true`로 포함하며 같은 장치 선택 옵션을 지원한다.

```bash
# 기본: PVQ30/PVQ33 φ1.6 카탈로그 곡선 모델
ros2 launch fss_bringup sim.launch.py

# 팔 사용, 추력기 미사용
ros2 launch fss_bringup sim.launch.py use_arm:=true use_thrusters:=false

# 이상 밸브: 명령을 즉시 선형 추력으로 변환
FSS_HYDRA_OVERRIDES="system/sim_valve=ideal" \
  ros2 launch fss_bringup sim.launch.py
```

포함:

- plant_model_node (3-DOF 동역학 + 밸브 모델 + 센서 모델)
- navigation (EKF)
- gnc (추력기와 GNC 옵션이 모두 true일 때)
- valve_driver (SimHal, 메인밸브 서비스·상태 발행)
- manager (모드 FSM)
- arm (`use_arm:=true`일 때)

## 런치 인자

| 인자 | 기본값 | 적용 런치 | 설명 |
|------|--------|----------|------|
| `use_sim` | `false` | fss_full, actuators | SimHal/PLCHal 전환 |
| `use_arm` | `false` | fss_full, sim | 팔 노드 실행 여부 |
| `use_thrusters` | `true` | fss_full, sim, actuators | 추력 출력 허용; false이면 전체 런치의 GNC도 생략 |
| `use_gnc` | `true` | fss_full, sim | 추력기 사용 시 GNC 실행 여부 |
| `use_imu` | `true` | sensors | IMU 브릿지 실행 여부 |
| `use_realsense` | `false` | sensors, fss_full | D435i 실행 여부 (EKF 입력과 분리) |
| `cpu_pin` | `false` | gnc, actuators, fss_full | CPU 코어 피닝 활성화 (taskset) |
| `cpu_core` | `4`/`5` | gnc, actuators | 노드를 실행할 CPU 코어 번호 |
| `rt_priority` | `0` | gnc, actuators | `1`–`99`: SCHED_FIFO 우선순위, `0`: 기존 스케줄링 유지 |
| `gnc_rt_priority` | `0` | fss_full, sim | GNC의 `rt_priority`로 전달 |
| `actuator_rt_priority` | `0` | fss_full, sim | 밸브 노드의 `rt_priority`로 전달 |
| `log_level` | `info` | gnc, actuators, fss_full | 전역 로깅 레벨 (debug/info/warn/error) |

`cpu_pin`은 CPU affinity만 설정하며 코어 독점이나 실시간 스케줄링 우선순위를 설정하지 않는다.
지정한 코어가 실행 환경에서 사용 가능한지 확인해야 한다.

### 실시간 스케줄링 선택

기본값의 원본은 `config/scheduling/default.yaml`이며, 단독·전체·SIL 런치가
모두 같은 Hydra 설정을 읽는다. 명시적인 런치 인자는 해당 실행의 YAML 기본값보다 우선한다.
기본 프로필은 피닝 OFF, RT 우선순위 0이며 부모 프로세스의 실행 정책을 그대로 사용한다.
이는 보드 실측 전의 기준 구성으로, 별도 설정 없이 RT 권한을 요구하지 않는다.

`config/scheduling/rt_trial.yaml`은 선택적으로 사용하는 A/B 시험 프로필이다.
GNC와 밸브를 같은 FIFO 30으로 시작하고 피닝은 OFF로 둔다. 30은 표준값이나 검증된
운용값이 아니다. 두 노드의 허용 지연·실행 시간이 미확정이므로 선후 관계를 강제하지
않는 출발점이며, 같은 FIFO 우선순위도 공정성이나 실행 순서를 보장하지 않는다.
항법(50 Hz)·GNC(20 Hz)·밸브 heartbeat(20 Hz)·감시(GNC와 같은 명목 20 Hz)의 주기만으로
우선순위를 확정할 수 없다. 통신 콜백과 IRQ/DDS 스레드도 함께 측정해야 한다.
이 프로필은 GNC/밸브에만 적용하며 항법·감시·센서 노드의 스케줄링은 바꾸지 않는다.

```bash
# 기본 기준 구성
python scripts/run_experiment.py system.use_sim=true

# RT 시험 구성; 실제 스케줄링 권한이 있는 Linux에서 실행
python scripts/run_experiment.py system.use_sim=true scheduling=rt_trial

# 직접 ros2 launch에서도 같은 프로필 사용
FSS_HYDRA_OVERRIDES="scheduling=rt_trial" ros2 launch fss_bringup sim.launch.py

# 보드에 맞게 조정하는 예 (검증된 운용값 아님)
python scripts/run_experiment.py system.use_sim=true scheduling=rt_trial \
  scheduling.gnc_rt_priority=35 scheduling.cpu_pin=true scheduling.gnc_cpu_core=2
```

`rt_priority`를 지정하면 해당 노드 실행 명령 앞에 `chrt -f <우선순위>`를 붙인다.
CPU 피닝과 독립적으로 사용할 수 있으며, 모두 기본값이면 기존 실행 방식을 유지한다.
이 옵션은 PREEMPT_RT를 설치하거나 켜지 않는다. SCHED_FIFO는 일반 Linux에서도
지원하며, PREEMPT_RT의 커널 선점 지연 개선은 별도로 준비한 커널에서 얻는다.

```bash
# 40은 형식 설명용 값이며 검증된 운용 우선순위가 아니다.
ros2 launch fss_bringup gnc.launch.py rt_priority:=40
ros2 launch fss_bringup actuators.launch.py cpu_pin:=true cpu_core:=5 rt_priority:=40
ros2 launch fss_bringup fss_full.launch.py use_sim:=true gnc_rt_priority:=40 actuator_rt_priority:=40
```

실행 환경에는 `chrt`/`taskset`(util-linux)이 필요하다. 실행 계정의 `RLIMIT_RTPRIO`
한도가 요청 우선순위를 허용하거나 `CAP_SYS_NICE` 권한이 있어야 한다.
범위를 벗어난 우선순위는 런치 오류로 거부하고, 도구·권한 부족 시 대상 노드 실행은
실패한다. 실패를 일반 스케줄링으로 조용히 대체하지 않는다.

적용 대상은 GNC/밸브 노드의 시작 스레드이며, 이후 생성되는 스레드는 라이브러리의
상속·설정 방식에 영향을 받는다. 특정 제어 콜백에만 우선순위를 부여하는 옵션이 아니다.
IRQ 스레드·통신·항법·관리자와의 관계를 고려해 값을 정하고, 실행 후 아래와 같이 확인한다.

```bash
ps -eLo pid,tid,cls,rtprio,psr,comm
chrt -p <TID>
taskset -pc <TID>
```

높은 우선순위 스레드가 계속 실행되면 다른 작업이 지연될 수 있다. 동기 PLC 통신이나
같은 executor의 콜백 대기는 이 옵션으로 해결되지 않는다. 보드에서 부하·통신 지연을
포함해 센서 입력부터 출력까지 측정해야 하며, 이 옵션만으로 deadline을 보장하지 않는다.

명령 조합·Hydra 프로필·우선순위 검증은 프로젝트의 Hydra 의존성이 설치된 환경에서
ROS 없이 다음으로 확인한다.

```bash
PYTHONPATH=src/infra/fss_common python3 -m unittest discover -s tests -p test_realtime_launch.py -v
```

`tests/test_realtime_launch_graph.py`는 ROS 2 `launch`가 설치된 환경에서 실제
LaunchService·Include·OpaqueFunction과 실행 명령 토큰화를 검사한다. 노드 실행만
기록용 Action으로 대체하며, 전체·SIL·단독 런치의 인자 전달과 명시적 인자 우선 적용을
확인한다. 실제 ROS 노드 실행이나 Linux 스케줄링 적용 검사는 포함하지 않는다.

### 실행 시간과 지터 측정

주기 T, 예정 시작 r_k=r_0+kT, 실제 시작 s_k, 종료 f_k를 같은 단조 시계로 기록한다.
시작 지연은 s_k-r_k, 주기 오차는 (s_k-s_(k-1))-T, 콜백 경과 시간은 f_k-s_k다.
deadline D가 정해지면 f_k-r_k>D인 비율도 구한다. 주기를 deadline으로 자동 간주하지 않는다.
콜백 경과 시간에는 CPU 선점·I/O 대기가 포함된다. Python의 `perf_counter_ns()`로
경과 시간을, `thread_time_ns()`로 현재 스레드의 CPU 사용 시간을 구분할 수 있다.
SIL의 ROS 시간은 배속·정지할 수 있으므로 실제 지연 측정 시 단조 시계와 혼합하지 않는다.

평균뿐 아니라 p99/p99.9, 관측 최댓값, deadline 초과 횟수와 측정 시간·부하를 남긴다.
관측 최댓값은 WCET 보장이 아니다. 제어 루프 안에서 매번 파일/콘솔에 쓰지 않고
측정값을 버퍼링해 내보내며, 계측 자체의 오버헤드도 비교한다.
센서 수신→항법→GNC→PLC 응답까지 단계별로 측정한다. PLC 응답은 실제 밸브 동작
완료와 같지 않으며, 물리 출력 지연은 계측기나 피드백이 필요하다.

CPU/네트워크/기록 부하, 통신 timeout, 열·전력 상태를 포함해 기본/RT 구성을 비교한다.
`cyclictest`/`rtla timerlat`는 OS 기상 지연 진단용이며 제어 전체 지연을 대신하지 않는다.
대기가 원인이면 통신·잠금·콜백 구조를, 계산이 원인이면 알고리즘·할당을 먼저 개선한다.

`use_thrusters:=false`가 `use_gnc:=true`보다 우선한다. 외부 추력 제어기를 사용하는 구성은
`use_thrusters:=true use_gnc:=false`다. 밸브 드라이버와 SIL 플랜트의 `use_thrusters`
파라미터는 read-only이며 옵션 변경은 재기동으로 적용한다.

`use_sim`은 팔의 `dry_run`을 변경하지 않는다. 실물 팔 설정은
[팔 명령 문서](../hardware/007_Arm_Commands.md)를 따른다.

## 규약

- **조건부 실행은 `IfCondition`/`UnlessCondition`으로만** — `LaunchConfiguration`은
  문자열을 반환하므로 `if use_sim`은 항상 참이 된다.
- **파라미터는 `fss_common.launch_utils.node_params("<노드명>")`로 로드** — Hydra
  트리에서 실행 시점에 YAML 생성. 장치·제어 설정은 기본값 없이 선언한다.
  `use_sim`, `use_sim_time`, `use_thrusters` 실행 플래그는 런치가 bool 파라미터로 주입한다.
  상세: [012_Configuration.md](012_Configuration.md).

오버라이드는 환경변수로:
```bash
FSS_HYDRA_OVERRIDES="gnc/controller=lqr system/sim_valve=ideal" \
  ros2 launch fss_bringup sim.launch.py
```

## Hydra 실험 러너

오버라이드를 자동으로 런치에 전달하고 컴포즈 결과를 출력:

```bash
python scripts/run_experiment.py experiment=default
python scripts/run_experiment.py experiment=station_keeping gnc/controller=lqr
python scripts/run_experiment.py system.use_sim=true system.use_arm=true system.use_thrusters=false
python scripts/run_experiment.py system.use_sim=true system.use_gnc=false
```

Hydra 오버라이드로 제어기, 유도 모드, 실험 파라미터를 커맨드라인에서 변경 가능.
러너의 `system.use_arm`, `system.use_thrusters`, `system.use_gnc`는 같은 이름의 런치 인자로
전달하며 기본값도 같다. 실행 플래그는 bool 값 `true`/`false`만 허용한다.
직접 `ros2 launch`로 실행할 때는 `use_arm:=true`와 같은 런치 인자를 사용한다.

## 장치 선택 검증

WSL Ubuntu 22.04 / ROS2 Humble에서 `fss_actuators`, `fss_sim`, `fss_bringup`을 빌드했다.
`tests/test_launch_options.py`는 전체 런치의 팔·추력기 네 조합, SIL 팔 구성,
옵션을 생략한 전체/SIL 기본 구성, 외부 제어기 구성(`use_gnc:=false`)을 검사한다.
실제 노드 선택, 시뮬레이션 시계, Hydra 파라미터 전달, 기동·종료를 확인한다.
`tests/test_run_experiment.py`는 실제 Hydra CLI와 기록용 ros2 대체 실행 파일로
기본 구성, 장치 선택, 제어기 오버라이드 전달 및 잘못된 플래그 거부를 검사한다.
`tests/test_native_thruster_options.py`의 검사는 실제 ROS 명령과 C++ SimHal을 사용해 추력 ON/OFF,
메인밸브 서비스·상태 유지, read-only 설정을 확인한다. 물리 장치 구동은 수행하지 않았다.
