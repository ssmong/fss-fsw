# 013. 본체·로봇팔 운용

## 개요

fss-fsw는 에어베어링 테스트베드의 센서, 항법, 본체 제어, 추력기와 로봇팔을 운용한다.
본체 GNC와 외부 관절 명령은 각각 해당 드라이버로 연결되며 시스템 manager가 모드를 관리한다.

| 구성 | 실행 코드 | 역할 |
|---|---|---|
| 전체 시스템 | `fss_bringup/fss_full.launch.py` | 서브시스템 기동 |
| 모의 시스템 | `fss_bringup/sim.launch.py` | 본체 SIL 구성 |
| 센서 | `fss_sensors` | Vicon·IMU 브리지 |
| 항법 | `fss_navigation/ekf_node` | 본체 상태 추정 |
| 본체 GNC | `fss_gnc/gnc_node` | 유도·제어·추력 할당 (Python 연구용은 `gnc_backend:=python`) |
| 밸브 | `fss_actuators/valve_driver_node` | PLC 출력·에어베어링 메인밸브 |
| 팔 | `fss_arm/arm_driver_node.py` | 위치·속도·전류 명령과 상태 피드백 |
| 관리자 | `fss_manager/manager_node` | 모드 전환·상태 감시 |

## 환경 준비

Ubuntu 22.04 / ROS2 Humble의 저장소 루트에서 실행한다.
WSL에서는 Windows의 `C:\Research\fss-fsw`가 `/mnt/c/Research/fss-fsw`에 대응한다.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/local_setup.bash
```

새 터미널마다 ROS와 install 환경을 적용한다.
`.venv-wsl`에 Hydra 등을 설치한 환경에서는 해당 패키지 경로도 적용한다.

```bash
export PYTHONPATH="$PWD/.venv-wsl/lib/python3.10/site-packages:${PYTHONPATH:-}"
```

## 장치 선택과 기동

| 옵션 | 기본값 | 동작 |
|---|---|---|
| `use_arm` | `false` | 팔 노드 실행 |
| `use_thrusters` | `true` | 추력 출력 허용 |
| `use_gnc` | `true` | 추력기 사용 시 본체 GNC 실행 |
| `use_sim` | `false` | 본체 SIL·SimHal 선택 |

팔과 추력기 옵션은 독립적이다. `use_thrusters:=false`이면 GNC 실행을 생략하고
밸브 드라이버의 추력 출력과 SIL 추력을 0으로 제한한다.
PLC 연결, 에어베어링 메인밸브 서비스, 상태 발행, 항법과 manager는 유지한다.
외부 추력 제어 프로그램을 사용할 때는 `use_thrusters:=true use_gnc:=false`를 사용한다.

```bash
# 본체 SIL
ros2 launch fss_bringup sim.launch.py

# 본체 SIL + 팔, 추력기 OFF
ros2 launch fss_bringup sim.launch.py use_arm:=true use_thrusters:=false

# 본체 SIL + 팔 + 추력기
ros2 launch fss_bringup sim.launch.py use_arm:=true use_thrusters:=true
```

팔은 `config/hardware/arm.yaml`의 `dry_run`으로 모의/실장치 연결을 별도 선택한다.
기본은 `dry_run: true`, `hardware_confirmed: false`다.
`use_sim`만 바꾸면 팔의 연결 설정은 바뀌지 않는다.

실제 본체 운용에는 Vicon 서버, PLC 및 Xsens 장치 설정이 필요하다.

```bash
# 별도 터미널: 외부 Xsens 드라이버
ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py

# 본체 센서·항법·밸브·manager와 선택한 팔/GNC
ros2 launch fss_bringup fss_full.launch.py use_arm:=true use_thrusters:=true
```

실제 팔 연결은 모델·ID·직렬 포트·영점·방향·감속비·한계를 확인한 후
`dry_run: false`, `hardware_confirmed: true`를 적용한다.
하드웨어 조건은 [팔 드라이버](../hardware/006_Arm.md)를 따른다.

## 시스템 모드 전환

각 서비스 응답의 `success: true`와 상태 반영을 확인하고 다음 단계로 진행한다.

```bash
# PLC 또는 SimHal 상태 수신 후 IDLE → PRESSURIZING
ros2 service call /system/set_mode fss_interfaces/srv/SetSystemMode "{requested_mode: 1}"

# 에어베어링 메인밸브 열기
ros2 service call /actuators/main_valve fss_interfaces/srv/MainValveControl "{open: true}"

# 메인밸브 열림 상태 반영 후 READY
ros2 service call /system/set_mode fss_interfaces/srv/SetSystemMode "{requested_mode: 2}"

# Vicon·항법 상태 준비 후 ACTIVE
ros2 service call /system/set_mode fss_interfaces/srv/SetSystemMode "{requested_mode: 3}"
```

거부 사유는 서비스 응답에 표시한다. 시스템 상태는 `/system/mode`와
`/system/health`에서 확인한다. 추력기를 꺼도 팔 운용에는 이 절차가 필요하다.

## 팔 명령과 시작

disable 상태에서 제어 모드를 선택하고, 별도 터미널의 발행자가 명령을 보내도록 준비한 뒤
팔을 enable한다. 관절 순서는 `[shoulder, elbow, wrist]`이며 최신 timestamp로 기본 100 Hz 발행한다.

```bash
ros2 service call /arm/set_enabled std_srvs/srv/SetBool "{data: false}"
ros2 service call /arm/set_command_mode fss_arm_msgs/srv/SetCommandMode "{mode: position}"

# 별도 터미널: 모의 실행용 목표 관절각
ros2 topic pub -r 100 -p 0 /arm/position_command sensor_msgs/msg/JointState \
  "{header: auto, name: [shoulder, elbow, wrist], position: [0.2, -0.2, 0.1]}"

# 다른 터미널: ACTIVE 수신과 발행자 준비 후 시작
ros2 service call /arm/set_enabled std_srvs/srv/SetBool "{data: true}"
```

위치·속도·전류의 전체 인터페이스와 모드 전환은
[팔 명령 가이드](../hardware/007_Arm_Commands.md)를 따른다.
명령이 100 ms 끊기거나 ACTIVE가 만료되면 팔은 fault로 정지한다.

## 상태 확인과 종료

| 토픽 | 내용 |
|---|---|
| `/system/mode`, `/system/health` | 모드·시스템 상태 |
| `/navigation/state` | 본체 추정 위치·속도 |
| `/actuators/valve_status` | 밸브 적용 명령·메인밸브 상태 |
| `/arm/joint_states`, `/arm/currents` | 팔 위치·속도·모터 전류 |
| `/arm/status`, `/arm/timing` | 팔 모드·fault·실행 주기 |

```bash
# 팔 정지 및 fault 해제
ros2 service call /arm/set_enabled std_srvs/srv/SetBool "{data: false}"

# 전체 비상정지: 추력과 에어베어링 메인밸브도 닫힘
ros2 service call /system/set_mode fss_interfaces/srv/SetSystemMode "{requested_mode: 5}"
```

팔 disable은 토크 해제이며 자세 유지 동작이 아니다.
기동 옵션 변경은 재시작으로 적용한다. 옵션·파라미터 상세는
[런치 문서](../infra/011_Bringup.md)를 따른다.

## 현재 검증 범위와 제한

단위 테스트, 공식 SDK의 모의 패킷 처리, 실제 ROS 토픽·서비스 및 SIL 기동·종료를 검사한다.
실제 모터의 추종 성능, 직렬 제어주기, 부하와 제동 성능은 장치 시험 대상이다.
본체 SIL과 팔 dry_run은 독립 모형이며 결합 동역학을 포함하지 않는다.

기존 본체 운용에서 다음 제한이 남아 있다.

- 밸브 드라이버는 ACTIVE 수신 후에만 추력을 허용하지만 모드·명령 만료 검사는 없다.
- ACTIVE 중 PLC 상태 소실·메인밸브 닫힘 및 팔 fault가 manager 중단 조건으로 연결되어 있지 않다.
- 팔의 ACTIVE freshness는 수신 시점 기준이며 모드 메시지의 지연·역순 시각 검사는 없다.
- GNC 포함 종료 과정의 ROS 구독 예외는 간헐적으로 관찰되었다.

이 항목들은 범용 운용 계층의 제한이며, 장치 선택 옵션이나 팔 단위 테스트의 통과가 이를 해소하지 않는다.
