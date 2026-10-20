# 로봇팔 위치·속도·전류 명령

## 지원 범위

`fss_arm`은 전 축에 공통으로 `velocity`, `position`, `current` 중 하나를 선택한다.
기본은 velocity이며 위치·전류는 Dynamixel의 실제 해당 제어 모드로 전송한다.
`fss_arm_msgs` 패키지는 전류 메시지와 모드 전환 서비스를 정의한다.

| 모드 | 토픽 | 메시지 / 값 | XM430 모드·목표 |
|---|---|---|---|
| velocity | `/arm/velocity_command` | `sensor_msgs/msg/JointState.velocity`, rad/s | Operating Mode 1, Goal Velocity(104), signed 4 byte |
| position | `/arm/position_command` | `sensor_msgs/msg/JointState.position`, rad | Operating Mode 4, Goal Position(116), signed 4 byte |
| current | `/arm/current_command` | `fss_arm_msgs/msg/JointCurrent.current`, A | Operating Mode 0, Goal Current(102), signed 2 byte |

위치는 보정된 관절 좌표계의 목표각이며 현재 각도에 더하는 증분이 아니다.
이름 순서는 항상 `[shoulder, elbow, wrist]`다. 위치/속도 JointState의 다른 명령 필드는
비운다. 전류는 **관절 +방향을 기준으로 부호를 맞춘 모터 전류 A**이며 토크 Nm가 아니다.
전류 변환에는 외부 감속비를 곱하지 않는다. 측정 전류는 `/arm/currents`에서 같은
`JointCurrent` 형식으로 발행하며 `/arm/joint_states.effort`는 비워 둔다.

모든 모드는 최신 timestamp를 붙여 기본 100 Hz로 계속 발행한다. 동일 위치를 유지할
때도 명령을 계속 보내야 한다. 일회성 목표를 무기한 유지하는 인터페이스는 아니다.
100 ms 명령 만료와 ACTIVE heartbeat 검사는 모든 모드에 적용한다.
enable 전에 발행되어 큐에 남은 신선한 명령은 폐기하며 새 enable 이후 명령만 사용한다.
미래/만료/역순 timestamp, 비유한 값, 관절 순서 및 한계 위반, 실행 모드와 다른 토픽은
정지 사유다. 제어 프로그램은 선택한 모드의 토픽으로 명령을 발행한다.

## 설정

`config/hardware/arm.yaml`:

```yaml
command_mode: velocity
current_limits_a: [0.2, 0.2, 0.2]
position_profile_velocity: [0.3, 0.3, 0.3]
position_profile_acceleration: [1.0, 1.0, 1.0]
```

이 수치는 모의 설정이다. 실장치의 ID·영점·회전 방향·외부 감속비·전류/위치 한계는
실제 팔에 맞게 확인해야 한다. 현재 기본 `dry_run: true`, `hardware_confirmed: false`다.

- 위치 profile 속도 단위는 rad/s, 가속도는 rad/s²다. 각각 속도·가속도 한계 이내여야 하며,
  모터 단위로 내림한 값이 0이면 거부한다. 장치의 profile 값 0은 무제한을 뜻한다.
- 위치 목표는 영점/부호/감속비와 관절 한계를 적용하며 ±π로 wrap하지 않는다.
- 전류 명령은 소프트웨어 `current_limits_a`와 장치 `Current Limit(38)`을 모두 검사한다.
  전류 명령 제한은 위치 모드의 전류 제한을 설정하는 기능이 아니다.
- current 모드에서는 측정 속도 한계와 현재 속도를 timeout까지 유지한 예상각도 검사한다.
  이는 제동거리·접촉력 보증이 아니다. 외부 토크 제어 또는 중력 보상은 구현하지 않았다.
- 시작 설정 파라미터는 read-only다. 런타임 모드 전환은 아래 서비스를 사용한다.

## 실행 예제

WSL Ubuntu 22.04 / ROS2 Humble, 저장소 루트에서 빌드한다.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select fss_arm_msgs fss_arm fss_bringup
source install/local_setup.bash
ros2 launch fss_bringup arm.launch.py
```

별도 FSS manager의 ACTIVE 상태가 필요하다. 본체 기동·모드 전환은
[실행 순서](../ops/013_Arm_Execution.md)를 따른다. 아래는 팔이 기동한 뒤의 명령이다.
각 서비스의 `success: true`를 확인하고 다음 단계로 진행한다.

전체 모의 시스템을 함께 띄울 때는 다음 런치를 사용한다. 이 경우 `arm.launch.py`는
별도로 실행하지 않는다.

```bash
ros2 launch fss_bringup fss_full.launch.py use_sim:=true use_arm:=true use_thrusters:=false
```

`use_arm:=false`는 팔 노드를 생략한다. `use_thrusters:=false`는 본체 추력을 차단하면서
에어베어링 메인밸브와 manager를 유지한다. 팔 실물/모의 연결은 `dry_run` 설정을 따른다.

### 위치 명령

```bash
# 1. 정지하고 모드 변경
ros2 service call /arm/set_enabled std_srvs/srv/SetBool "{data: false}"
ros2 service call /arm/set_command_mode fss_arm_msgs/srv/SetCommandMode "{mode: position}"

# 2. 별도 터미널에서 위치 목표를 계속 발행 (rad)
ros2 topic pub -r 100 -p 0 /arm/position_command sensor_msgs/msg/JointState \
  "{header: auto, name: [shoulder, elbow, wrist], position: [0.2, -0.2, 0.1]}"

# 3. 다른 터미널에서 ACTIVE 수신 후 시작
ros2 service call /arm/set_enabled std_srvs/srv/SetBool "{data: true}"
```

### 전류 명령

기존 위치/속도 발행자를 종료한 뒤 다음을 실행한다.

```bash
ros2 service call /arm/set_enabled std_srvs/srv/SetBool "{data: false}"
ros2 service call /arm/set_command_mode fss_arm_msgs/srv/SetCommandMode "{mode: current}"

# 별도 터미널: shoulder에 +0.05 A, 나머지 0 A
ros2 topic pub -r 100 -p 0 /arm/current_command fss_arm_msgs/msg/JointCurrent \
  "{header: auto, name: [shoulder, elbow, wrist], current: [0.05, 0.0, 0.0]}"

# 다른 터미널에서 시작
ros2 service call /arm/set_enabled std_srvs/srv/SetBool "{data: true}"
```

속도 모드로 돌아가려면 disable 후 `SetCommandMode "{mode: velocity}"`를 호출한다.
실제 선택 모드는 `/arm/status`의 `mode`에 표시한다. `command_mode` 파라미터는 시작
설정이며 런타임 서비스 전환 후 현재 모드를 나타내는 값이 아니다.

## 시작·전환·정지 동작

- 모드 전환은 disable 및 fault 해제 상태에서만 허용한다. 하드웨어는 전 모터의
  torque OFF, watchdog/error 해제, 정지 속도를 확인한 뒤 Operating Mode를 기록한다.
- 모드 전환으로 초기화되는 속도 PI·위치 PID·feedforward·Goal PWM 설정은 전환 전
  값으로 복원한다. 영점은 자동 보정하지 않는다.
- 모드 전환 전후 위치 카운터가 2 tick을 초과해 달라지면 실패 처리한다. 일부 모터만
  전환된 경우에도 enable을 허용하지 않는다. 위치 카운터/영점을 확인하고 재시작해야 한다.
- 위치 enable은 현재 측정 위치를 전 축 Goal Position에 쓰고 읽어 확인한 뒤 토크를 켠다.
  토크 enable 후에도 카운터 연속성을 확인한다. 기존 목표나 원점으로 자동 이동하지 않는다.
- 속도·전류 enable은 각각 0 rad/s·0 A를 확인한 뒤 토크를 켠다.
- disable/fault는 속도·전류 모드에서 0 목표 후 torque OFF, 위치 모드에서는 torque OFF다.
  위치 모드 정지에 Goal Position=0을 보내지 않는다. 토크 OFF는 자세 유지가 아니다.

## 모의 검증 범위

속도 dry_run은 양자화 속도를 적분한다. 위치 dry_run은 목표까지 profile 속도로 제한한
이상적 이동이며 실제 PID·가속도 profile·부하를 재현하지 않는다. 전류 dry_run은 명령을
양자화해 전류 피드백으로 보여주고 전류에 의한 움직임은 생성하지 않는다.

공식 SDK와 가짜 모터/바이트 포트를 사용해 목표 주소·단위·모드 전환·목표 반영·
오류·부분 응답을 검사한다. 실제 ROS 토픽/서비스로 위치·전류 입력, 모드 혼입,
한계 초과와 timeout 정지를 검사한다. 물리 모터 구동은 수행하지 않았다.

검증 환경은 WSL Ubuntu 22.04 / ROS2 Humble이다. 팔 드라이버 검사는 `tests/arm/`,
전체 장치 선택 검사는 `tests/test_launch_options.py`와 `tests/test_native_thruster_options.py`에 있다.

제어표 근거: [ROBOTIS XM430-W350](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/).
