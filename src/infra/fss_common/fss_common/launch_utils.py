"""런치 파일용 파라미터 생성 유틸리티.

모든 런치 파일은 node_params()로 Hydra 트리(config/)에서 노드별
ROS2 파라미터 YAML을 생성해 로드한다. 인라인 파라미터 dict 금지.

Hydra 오버라이드는 FSS_HYDRA_OVERRIDES 환경변수(공백 구분)로 전달된다.
(run_experiment.py가 설정; 직접 실행 예:
  FSS_HYDRA_OVERRIDES="gnc/controller=lqr" ros2 launch fss_bringup gnc.launch.py)
"""

import logging
import os
import shlex
import tempfile

logger = logging.getLogger(__name__)

_ENV_OVERRIDES = "FSS_HYDRA_OVERRIDES"


def hydra_overrides_from_env() -> list[str]:
    """환경변수에서 Hydra 오버라이드 목록을 읽는다."""
    return shlex.split(os.environ.get(_ENV_OVERRIDES, ""))


def execution_prefix(cpu_pin: bool, cpu_core: str, rt_priority: str) -> str:
    """CPU affinity와 선택적 SCHED_FIFO 실행 명령을 구성한다. 0은 RT 미설정."""
    try:
        priority = int(rt_priority)
    except ValueError as exc:
        raise ValueError("rt_priority must be an integer from 0 to 99") from exc
    if not 0 <= priority <= 99:
        raise ValueError("rt_priority must be an integer from 0 to 99")
    command = ["chrt", "-f", str(priority)] if priority else []
    if cpu_pin:
        command.extend(["taskset", "-c", cpu_core])
    return shlex.join(command)


def scheduling_defaults() -> dict[str, str]:
    """Hydra scheduling 프로필을 검증하고 런치 기본값으로 변환한다."""
    from fss_common.hydra_bridge import load_hydra_config
    from fss_common.paths import config_dir

    cfg = load_hydra_config(
        config_dir(), overrides=hydra_overrides_from_env()
    ).scheduling
    if type(cfg.cpu_pin) is not bool:
        raise ValueError("scheduling.cpu_pin must be true or false")
    defaults = {"cpu_pin": str(cfg.cpu_pin).lower()}
    for name in ("gnc_cpu_core", "actuator_cpu_core", "gnc_rt_priority", "actuator_rt_priority"):
        value = cfg[name]
        if type(value) is not int or value < 0:
            raise ValueError(f"scheduling.{name} must be a non-negative integer")
        if name.endswith("rt_priority") and value > 99:
            raise ValueError(f"scheduling.{name} must be in 0..99")
        defaults[name] = str(value)
    return defaults


def scheduled_node(context, **node_kwargs):
    """런치 인자를 실행 시점에 평가해 GNC/밸브 노드에 적용한다."""
    from launch.conditions import IfCondition
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node

    prefix = execution_prefix(
        IfCondition(LaunchConfiguration("cpu_pin")).evaluate(context),
        LaunchConfiguration("cpu_core").perform(context),
        LaunchConfiguration("rt_priority").perform(context),
    )
    return [Node(prefix=prefix, **node_kwargs)]


def node_params(node_name: str) -> str:
    """노드의 ROS2 파라미터 YAML을 생성하고 경로를 반환한다.

    실패(config 트리/노드 키 누락, 잘못된 오버라이드)는 예외로 전파되어
    런치 자체가 명시적으로 실패한다.
    """
    from fss_common.hydra_bridge import generate_node_param_files

    overrides = hydra_overrides_from_env()
    out_dir = tempfile.mkdtemp(prefix=f"fss_params_{node_name}_")
    files = generate_node_param_files([node_name], out_dir, overrides=overrides)
    path = str(files[node_name])
    logger.info("[fss_common.launch_utils] %s 파라미터 생성: %s", node_name, path)
    return path
