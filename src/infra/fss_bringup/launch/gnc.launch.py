"""GNC 서브시스템 런치 파일.

파라미터는 Hydra 트리(config/)에서 생성 — 인라인 dict 금지 (SSoT).
오버라이드: FSS_HYDRA_OVERRIDES="gnc/controller=lqr" ros2 launch ...

사용법:
  ros2 launch fss_bringup gnc.launch.py
  ros2 launch fss_bringup gnc.launch.py cpu_pin:=true cpu_core:=4
  ros2 launch fss_bringup gnc.launch.py gnc_backend:=python  # 연구용, bindings 빌드 필요
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

from fss_common.launch_utils import node_params, scheduled_node, scheduling_defaults


def launch_gnc(context, **node_kwargs):
    """기본 C++ 노드와 명시적으로 선택한 Python 연구 노드를 실행한다."""
    backend = LaunchConfiguration("gnc_backend").perform(context)
    if backend not in ("cpp", "python"):
        raise ValueError("gnc_backend must be cpp or python")
    return scheduled_node(
        context, executable="gnc_node" if backend == "cpp" else "gnc_node.py", **node_kwargs
    )


def generate_launch_description() -> LaunchDescription:
    """런치 설명을 생성한다."""

    scheduling = scheduling_defaults()
    backend_arg = DeclareLaunchArgument(
        "gnc_backend", default_value="cpp", choices=["cpp", "python"],
        description="기본 C++ 런타임 또는 Python 연구용 GNC (bindings 빌드 필요)",
    )

    cpu_pin_arg = DeclareLaunchArgument(
        "cpu_pin", default_value=scheduling["cpu_pin"],
        description="CPU 코어 피닝 활성화",
    )
    cpu_core_arg = DeclareLaunchArgument(
        "cpu_core", default_value=scheduling["gnc_cpu_core"],
        description="GNC 노드를 실행할 CPU 코어 번호 (Jetson 6코어 중)",
    )
    rt_priority_arg = DeclareLaunchArgument(
        "rt_priority", default_value=scheduling["gnc_rt_priority"],
        description="SCHED_FIFO 우선순위 (1-99), 0이면 기존 스케줄링 유지",
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level", default_value="info",
        description="로깅 레벨 (debug/info/warn/error)",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="SIL에서 /clock 사용 여부",
    )

    log_level = LaunchConfiguration("log_level")

    # Hydra → ROS2 파라미터 파일 생성 (실패 시 런치 실패)
    params = [
        node_params("gnc_node"),
        {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)},
    ]

    gnc_node = OpaqueFunction(
        function=launch_gnc,
        kwargs=dict(
            package="fss_gnc",
            name="gnc_node",
            parameters=params,
            arguments=["--ros-args", "--log-level", log_level],
            output="screen",
        ),
    )

    return LaunchDescription([
        backend_arg,
        cpu_pin_arg,
        cpu_core_arg,
        rt_priority_arg,
        log_level_arg,
        use_sim_time_arg,
        gnc_node,
    ])
