"""실장치 없이 실험 프리셋의 장치 선택과 CLI 우선순위를 확인한다."""

import runpy
from pathlib import Path

import pytest

from fss_common.hydra_bridge import load_hydra_config

REPO = Path(__file__).resolve().parents[1]
launch_command = runpy.run_path(str(REPO / "scripts/run_experiment.py"))["_launch_command"]


@pytest.mark.parametrize(
    ("overrides", "gnc_enabled"),
    [
        ([], True),
        (["experiment=thrust_characterization"], False),
        (["experiment=thrust_characterization", "system.use_gnc=true"], True),
    ],
)
def test_experiment_controls_gnc_without_disabling_external_thruster_commands(
    overrides, gnc_enabled,
):
    config = load_hydra_config(REPO / "config", overrides=overrides)
    command = launch_command(config)
    assert f"use_gnc:={str(gnc_enabled).lower()}" in command
    assert "use_thrusters:=true" in command


@pytest.mark.parametrize(
    ("preset", "controller", "guidance"),
    [
        ("station_keeping", "pid", "target"),
        ("trajectory_tracking", "lqr", "trajectory"),
    ],
)
def test_existing_experiments_keep_their_controller_and_guidance(preset, controller, guidance):
    config = load_hydra_config(REPO / "config", overrides=[f"experiment={preset}"])
    assert config.gnc_node.translational.type == controller
    assert config.gnc_node.guidance.mode == guidance
