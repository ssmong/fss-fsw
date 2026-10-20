"""Hydra 설정 → ROS2 파라미터 YAML 브릿지.

config/ Hydra 트리가 유일한 설정 소스(SSoT)다.
런치 파일과 run_experiment.py 모두 generate_node_param_files()로
노드별 ROS2 파라미터 파일을 생성해 사용한다.

실패는 모두 예외로 전파한다 (silent fallback 금지).
"""

import logging
from pathlib import Path
from typing import Any

import yaml
from omegaconf import DictConfig, OmegaConf

from fss_common.paths import config_dir as runtime_config_dir

logger = logging.getLogger(__name__)


def load_hydra_config(
    config_path: str | Path,
    config_name: str = "config",
    overrides: list[str] | None = None,
) -> DictConfig:
    """Hydra Compose API로 설정을 로드한다.

    Raises:
        Exception: 설정 디렉토리/파일 누락, 잘못된 오버라이드 등 Hydra 오류
    """
    from hydra import compose, initialize_config_dir

    config_dir = Path(config_path).resolve()
    if not config_dir.is_dir():
        raise FileNotFoundError(f"[fss_common.hydra_bridge] 설정 디렉토리 없음: {config_dir}")

    with initialize_config_dir(config_dir=str(config_dir), version_base=None):
        return compose(config_name=config_name, overrides=overrides or [])


def hydra_to_ros2_params(cfg: DictConfig, node_name: str) -> dict[str, Any]:
    """설정 트리에서 노드 서브트리를 추출해 일반 딕셔너리로 반환한다.

    Raises:
        KeyError: 설정 트리에 node_name 키가 없는 경우
    """
    container = OmegaConf.to_container(cfg, resolve=True)
    if not isinstance(container, dict) or node_name not in container:
        top = sorted(container.keys()) if isinstance(container, dict) else type(container)
        raise KeyError(
            f"[fss_common.hydra_bridge] 설정 트리에 노드 '{node_name}' 없음 (최상위 키: {top})"
        )
    params = container[node_name]
    if not isinstance(params, dict):
        raise TypeError(
            f"[fss_common.hydra_bridge] '{node_name}' 항목이 딕셔너리가 아님: {type(params)}"
        )
    return params


def write_ros2_param_yaml(params: dict[str, Any], node_name: str, output_path: str | Path) -> Path:
    """`노드명: ros__parameters:` 형식의 ROS2 파라미터 파일을 쓴다."""
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    with open(output, "w", encoding="utf-8") as f:
        yaml.safe_dump(
            {node_name: {"ros__parameters": params}},
            f, default_flow_style=False, allow_unicode=True, sort_keys=False,
        )
    return output


def generate_node_param_files(
    node_names: list[str],
    out_dir: str | Path,
    overrides: list[str] | None = None,
    config_dir: str | Path | None = None,
) -> dict[str, Path]:
    """Hydra 트리를 컴포즈하여 노드별 ROS2 파라미터 파일을 생성한다.

    Args:
        node_names: 생성 대상 노드명 (설정 트리 최상위 키와 일치해야 함)
        out_dir: 출력 디렉토리
        overrides: Hydra 오버라이드 (예: ["gnc/controller=lqr"])
        config_dir: config/ 경로 (기본: 환경변수/설치 패키지/소스 트리에서 탐색)

    Returns:
        {노드명: 생성된 YAML 경로}

    Raises:
        KeyError/FileNotFoundError 등: 어떤 노드라도 생성 실패 시 (부분 성공 없음)
    """
    if config_dir is None:
        config_dir = runtime_config_dir()
    cfg = load_hydra_config(config_dir, overrides=overrides)

    result: dict[str, Path] = {}
    for node in node_names:
        params = hydra_to_ros2_params(cfg, node)
        result[node] = write_ros2_param_yaml(params, node, Path(out_dir) / f"{node}.yaml")
    logger.info(
        "[fss_common.hydra_bridge] 파라미터 파일 %d개 생성: %s (overrides=%s)",
        len(result), out_dir, overrides or [],
    )
    return result
