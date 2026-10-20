"""액추에이터 서브시스템 런치 파일.

파라미터는 Hydra 트리(config/hardware/plc.yaml)에서 생성 — SSoT.
use_sim과 use_thrusters는 실행 플래그로 주입한다 (HAL 선택 및 추력 출력 허용).

사용법:
  ros2 launch fss_bringup actuators.launch.py
  ros2 launch fss_bringup actuators.launch.py cpu_pin:=true cpu_core:=5
"""

from fss_common.launch_utils import node_params, scheduled_node, scheduling_defaults
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """런치 설명을 생성한다."""

    scheduling = scheduling_defaults()

    use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="시뮬레이션 모드 (true: SimHal, false: PLC)",
    )
    use_thrusters_arg = DeclareLaunchArgument(
        "use_thrusters",
        default_value="true",
        description="추력 출력 허용 여부; false여도 메인밸브 서비스와 상태 발행 유지",
    )
    cpu_pin_arg = DeclareLaunchArgument(
        "cpu_pin",
        default_value=scheduling["cpu_pin"],
        description="CPU 코어 피닝 활성화",
    )
    cpu_core_arg = DeclareLaunchArgument(
        "cpu_core",
        default_value=scheduling["actuator_cpu_core"],
        description="밸브 드라이버를 실행할 CPU 코어 번호",
    )
    rt_priority_arg = DeclareLaunchArgument(
        "rt_priority", default_value=scheduling["actuator_rt_priority"],
        description="SCHED_FIFO 우선순위 (1-99), 0이면 기존 스케줄링 유지",
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="로깅 레벨 (debug/info/warn/error)",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="SIL에서 /clock 사용 여부",
    )

    use_sim = LaunchConfiguration("use_sim")
    log_level = LaunchConfiguration("log_level")

    params = [
        node_params("valve_driver_node"),
        {
            "use_sim": ParameterValue(use_sim, value_type=bool),
            "use_thrusters": ParameterValue(LaunchConfiguration("use_thrusters"), value_type=bool),
            "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
        },
    ]

    valve_driver = OpaqueFunction(
        function=scheduled_node,
        kwargs=dict(
            package="fss_actuators",
            executable="valve_driver_node",
            name="valve_driver_node",
            parameters=params,
            arguments=["--ros-args", "--log-level", log_level],
            output="screen",
        ),
    )

    return LaunchDescription(
        [
            use_sim_arg,
            use_thrusters_arg,
            cpu_pin_arg,
            cpu_core_arg,
            rt_priority_arg,
            log_level_arg,
            use_sim_time_arg,
            valve_driver,
        ]
    )
