"""FSS 전체 시스템 런치 파일.

실행 서브시스템:
- 액추에이터 (밸브 드라이버 — use_sim 시 SimHal)
- 센서 (Vicon + Xsens, D435i 선택; 실HW에서만)
- SIL 플랜트 (use_sim 시에만 — /clock 발행)
- 항법 (EKF), 선택 GNC·로봇팔, 관리자

장치·제어 설정은 Hydra 트리(config/)에서 생성하고 실행 플래그는 런치에서 주입한다.
IDLE 기동 후 /system/set_mode 서비스로 전이한다.
밸브의 모드 미수신·명령 만료 제한은 docs/ops/013_Arm_Execution.md를 참고한다.

사용법:
  ros2 launch fss_bringup fss_full.launch.py
  ros2 launch fss_bringup fss_full.launch.py use_sim:=true
  ros2 launch fss_bringup fss_full.launch.py use_sim:=true use_arm:=true use_thrusters:=false
  ros2 launch fss_bringup fss_full.launch.py use_realsense:=true
  ros2 launch fss_bringup fss_full.launch.py cpu_pin:=true log_level:=debug
"""

from fss_common.launch_utils import node_params, scheduling_defaults
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """런치 설명을 생성한다."""

    scheduling = scheduling_defaults()

    bringup_share = FindPackageShare("fss_bringup")

    # ── 런치 인자 ──
    use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="시뮬레이션 모드 (true: SIL 플랜트+SimHal, false: 실제 하드웨어)",
    )
    cpu_pin_arg = DeclareLaunchArgument(
        "cpu_pin",
        default_value=scheduling["cpu_pin"],
        description="GNC/밸브 노드 CPU 코어 피닝 활성화",
    )
    gnc_rt_priority_arg = DeclareLaunchArgument(
        "gnc_rt_priority", default_value=scheduling["gnc_rt_priority"],
        description="GNC SCHED_FIFO 우선순위 (1-99), 0이면 기존 스케줄링 유지",
    )
    actuator_rt_priority_arg = DeclareLaunchArgument(
        "actuator_rt_priority", default_value=scheduling["actuator_rt_priority"],
        description="밸브 SCHED_FIFO 우선순위 (1-99), 0이면 기존 스케줄링 유지",
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="전역 로깅 레벨 (debug/info/warn/error)",
    )
    use_realsense_arg = DeclareLaunchArgument(
        "use_realsense",
        default_value="false",
        description="RealSense D435i 선택 실행 (EKF 입력에는 연결하지 않음)",
    )

    use_sim = LaunchConfiguration("use_sim")
    use_arm_arg = DeclareLaunchArgument(
        "use_arm", default_value="false", description="로봇팔 위치·속도·전류 명령 노드 실행"
    )
    use_thrusters_arg = DeclareLaunchArgument(
        "use_thrusters",
        default_value="true",
        description="추력기 사용 (false: GNC 생략, 추력 출력 차단; 에어베어링 유지)",
    )
    use_thrusters = LaunchConfiguration("use_thrusters")
    use_gnc_arg = DeclareLaunchArgument(
        "use_gnc", default_value="true", description="추력기 사용 시 기존 본체 GNC 실행"
    )
    gnc_backend_arg = DeclareLaunchArgument(
        "gnc_backend", default_value="cpp", choices=["cpp", "python"],
        description="기본 C++ GNC 또는 Python 연구용 GNC (bindings 빌드 필요)",
    )
    arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "arm.launch.py"])
        ),
        condition=IfCondition(LaunchConfiguration("use_arm")),
    )
    cpu_pin = LaunchConfiguration("cpu_pin")
    log_level = LaunchConfiguration("log_level")
    use_realsense = LaunchConfiguration("use_realsense")

    # ── 액추에이터 ──
    actuators_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "actuators.launch.py"])
        ),
        launch_arguments={
            "use_sim": use_sim,
            "use_thrusters": use_thrusters,
            "use_sim_time": use_sim,
            "cpu_pin": cpu_pin,
            "cpu_core": scheduling["actuator_cpu_core"],
            "rt_priority": LaunchConfiguration("actuator_rt_priority"),
            "log_level": log_level,
        }.items(),
    )

    # ── 센서 (실제 하드웨어일 때만) ──
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "sensors.launch.py"])
        ),
        launch_arguments={
            "use_imu": "true",
            "use_realsense": use_realsense,
        }.items(),
        condition=UnlessCondition(use_sim),
    )

    # ── SIL 플랜트 (시뮬레이션일 때만 — 센서 토픽 + /clock 공급) ──
    plant_node = Node(
        package="fss_sim",
        executable="plant_model_node",
        name="plant_model_node",
        parameters=[
            node_params("plant_model_node"),
            {"use_thrusters": ParameterValue(use_thrusters, value_type=bool)},
        ],
        output="screen",
        condition=IfCondition(use_sim),
    )

    # ── 항법 (EKF) ──
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "navigation.launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim}.items(),
    )

    # ── GNC (유도 + 제어 + 할당) ──
    gnc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "gnc.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim,
            "gnc_backend": LaunchConfiguration("gnc_backend"),
            "cpu_pin": cpu_pin,
            "cpu_core": scheduling["gnc_cpu_core"],
            "rt_priority": LaunchConfiguration("gnc_rt_priority"),
            "log_level": log_level,
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_gnc")),
    )

    # ── 관리자 (모드 FSM + 건강 감시) ──
    manager_node = Node(
        package="fss_manager",
        executable="manager_node",
        name="manager_node",
        parameters=[
            node_params("manager_node"),
            {"use_sim_time": ParameterValue(use_sim, value_type=bool)},
        ],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
    )

    return LaunchDescription(
        [
            use_sim_arg,
            use_arm_arg,
            use_thrusters_arg,
            use_gnc_arg,
            gnc_backend_arg,
            cpu_pin_arg,
            gnc_rt_priority_arg,
            actuator_rt_priority_arg,
            log_level_arg,
            use_realsense_arg,
            actuators_launch,
            arm_launch,
            sensors_launch,
            plant_node,
            navigation_launch,
            GroupAction(actions=[gnc_launch], condition=IfCondition(use_thrusters)),
            manager_node,
        ]
    )
