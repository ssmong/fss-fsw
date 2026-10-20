#!/usr/bin/env python3
"""Hydra 기반 실험 실행 스크립트.

Hydra 설정을 해석·검증하고, CLI 오버라이드를 FSS_HYDRA_OVERRIDES 환경변수로
ros2 launch에 전달한다. 런치 파일이 같은 Hydra 트리에서 노드별 파라미터를
생성하므로 설정 소스는 config/ 하나뿐이다 (SSoT).
종료는 Ctrl+C로 하고, rosbag 기록은 별도 터미널에서 수동으로 시작·종료한다.
experiment.duration_sec/record_bag는 메타데이터이며 현재 실행에는 적용하지 않는다.

사용 예:
    # 기본 실험
    python scripts/run_experiment.py

    # 궤적 추종 실험 (LQR)
    python scripts/run_experiment.py experiment=trajectory_tracking

    # 개별 파라미터 오버라이드
    python scripts/run_experiment.py gnc/controller=lqr gnc_node.translational.kp=8.0

"""

import os
import shlex
import subprocess
import sys
from pathlib import Path

import hydra
from hydra.core.hydra_config import HydraConfig
from omegaconf import DictConfig, OmegaConf

CONFIG_DIR = Path(
    os.environ.get("FSS_CONFIG_DIR", Path(__file__).resolve().parents[1] / "config")
).expanduser().resolve()


def _launch_command(cfg: DictConfig) -> list[str]:
    """장치 선택 플래그를 검증하고 ROS2 런치 명령으로 변환한다."""
    flags = {}
    for name in ("use_sim", "use_arm", "use_thrusters", "use_gnc"):
        value = cfg.system[name]
        if not isinstance(value, bool):
            raise ValueError(f"system.{name}은 true/false여야 합니다: {value!r}")
        flags[name] = value
    launch_file = "sim.launch.py" if flags["use_sim"] else "fss_full.launch.py"
    command = ["ros2", "launch", "fss_bringup", launch_file]
    if not flags["use_sim"]:
        command.append("use_sim:=false")
    command.extend(f"{name}:={str(flags[name]).lower()}" for name in flags if name != "use_sim")
    backend = cfg.system.gnc_backend
    if backend not in ("cpp", "python"):
        raise ValueError(f"system.gnc_backend must be cpp or python: {backend!r}")
    command.append(f"gnc_backend:={backend}")
    return command


@hydra.main(config_path=str(CONFIG_DIR), config_name="config", version_base=None)
def run_experiment(cfg: DictConfig) -> None:
    """Hydra 설정을 해석하고 ROS2 런치를 실행한다.

    Args:
        cfg: 해석된 Hydra 설정 (오버라이드 적용 후 — 여기서 오류면 즉시 실패)
    """
    print("=" * 60)
    print("FSS 실험 설정 (해석 완료)")
    print("=" * 60)
    print(OmegaConf.to_yaml(cfg))
    print("=" * 60)

    experiment_name = cfg.experiment.name
    cmd = _launch_command(cfg)
    use_sim = cfg.system.use_sim
    print(f"실험: {experiment_name}")
    print(f"시뮬레이션 모드: {use_sim}")

    # 이 실행에 적용된 CLI 오버라이드를 런치(파라미터 생성기)로 전달
    overrides = list(HydraConfig.get().overrides.task)
    env = dict(os.environ)
    env["FSS_HYDRA_OVERRIDES"] = " ".join(shlex.quote(o) for o in overrides)
    # Hydra CLI와 C++/Python 노드의 파일 기반 설정을 동일한 트리에 고정한다.
    env["FSS_CONFIG_DIR"] = str(CONFIG_DIR)

    print(f"\n실행 명령: {' '.join(cmd)}")
    if overrides:
        print(f"Hydra 오버라이드: {overrides}")
    print("=" * 60)

    try:
        subprocess.run(cmd, check=True, env=env)
    except subprocess.CalledProcessError as e:
        print(f"런치 실행 실패: {e}", file=sys.stderr)
        sys.exit(1)
    except FileNotFoundError:
        print(
            "ros2 명령을 찾을 수 없습니다. ROS2 환경을 source 했는지 확인하세요.",
            file=sys.stderr,
        )
        sys.exit(1)


if __name__ == "__main__":
    run_experiment()
