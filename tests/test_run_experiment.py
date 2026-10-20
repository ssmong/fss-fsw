"""실제 Hydra CLI의 기본값·장치 선택·오버라이드 전달을 하드웨어 없이 검사한다."""

import json
import os
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]


@pytest.fixture
def runner_env(tmp_path: Path) -> tuple[dict[str, str], Path]:
    """모든 CLI 검사의 ros2 호출을 기록만 하는 실행 파일로 대체한다."""
    capture = tmp_path / "invocation.json"
    ros2 = tmp_path / "ros2"
    ros2.write_text(
        "#!/usr/bin/env python3\n"
        "import json, os, pathlib, sys\n"
        "pathlib.Path(os.environ['FSS_TEST_CAPTURE']).write_text(json.dumps({\n"
        "    'args': sys.argv[1:], 'overrides': os.environ['FSS_HYDRA_OVERRIDES'],\n"
        "    'config_dir': os.environ['FSS_CONFIG_DIR']}))\n",
        encoding="utf-8",
    )
    ros2.chmod(0o755)
    env = dict(os.environ)
    env.pop("FSS_CONFIG_DIR", None)
    env["PATH"] = f"{tmp_path}{os.pathsep}{env['PATH']}"
    env["FSS_TEST_CAPTURE"] = str(capture)
    return env, capture


@pytest.mark.parametrize(
    ("overrides", "launch_file", "flags"),
    [
        ([], "fss_full.launch.py", (False, True, True)),
        (["system.use_sim=true"], "sim.launch.py", (False, True, True)),
        (
            ["system.use_sim=true", "scheduling=rt_trial", "scheduling.gnc_rt_priority=35"],
            "sim.launch.py",
            (False, True, True),
        ),
        (
            ["system.use_sim=true", "system.use_arm=true", "system.use_thrusters=false"],
            "sim.launch.py",
            (True, False, True),
        ),
        (
            ["system.use_arm=true", "system.use_gnc=false"],
            "fss_full.launch.py",
            (True, True, False),
        ),
        (["system.use_thrusters=false"], "fss_full.launch.py", (False, False, True)),
    ],
)
@pytest.mark.parametrize("backend", ["cpp", "python"])
def test_hydra_cli_forwards_device_selection(
    tmp_path: Path,
    runner_env: tuple[dict[str, str], Path],
    overrides: list[str],
    launch_file: str,
    flags: tuple[bool, bool, bool],
    backend: str,
) -> None:
    """ros2 실행 경계만 대체해 실제 CLI 설정과 subprocess 인자를 확인한다."""
    env, capture = runner_env
    overrides = [*overrides, "gnc/controller=lqr", "gnc_node.translational.kp=8.0"]
    if backend == "python":
        overrides.append("system.gnc_backend=python")
    result = subprocess.run(
        [
            sys.executable,
            str(REPO / "scripts" / "run_experiment.py"),
            *overrides,
            f"hydra.run.dir={tmp_path / 'hydra'}",
        ],
        cwd=REPO,
        env=env,
        capture_output=True,
        text=True,
        timeout=20,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    invocation = json.loads(capture.read_text(encoding="utf-8"))
    expected = ["launch", "fss_bringup", launch_file]
    if launch_file == "fss_full.launch.py":
        expected.append("use_sim:=false")
    expected.extend(
        f"{name}:={str(value).lower()}"
        for name, value in zip(("use_arm", "use_thrusters", "use_gnc"), flags, strict=True)
    )
    expected.append(f"gnc_backend:={backend}")
    assert invocation["args"] == expected
    assert shlex.split(invocation["overrides"]) == overrides
    assert Path(invocation["config_dir"]) == REPO / "config"


@pytest.mark.parametrize("name", ["use_sim", "use_arm", "use_thrusters", "use_gnc"])
def test_invalid_device_flag_is_rejected(
    tmp_path: Path, runner_env: tuple[dict[str, str], Path], name: str
) -> None:
    """문자열을 bool로 강제 변환해 잘못된 장치를 켜지 않아야 한다."""
    env, capture = runner_env
    result = subprocess.run(
        [
            sys.executable,
            str(REPO / "scripts" / "run_experiment.py"),
            f"system.{name}=invalid",
            f"hydra.run.dir={tmp_path / 'hydra'}",
        ],
        cwd=REPO,
        env=env,
        capture_output=True,
        text=True,
        timeout=20,
    )
    assert result.returncode != 0
    assert f"system.{name}은 true/false" in result.stderr
    assert not capture.exists()


def test_invalid_gnc_backend_is_rejected(tmp_path: Path, runner_env) -> None:
    env, capture = runner_env
    result = subprocess.run(
        [sys.executable, str(REPO / "scripts/run_experiment.py"),
         "system.gnc_backend=invalid", f"hydra.run.dir={tmp_path / 'hydra'}"],
        cwd=REPO, env=env, capture_output=True, text=True, timeout=20,
    )
    assert result.returncode != 0
    assert "system.gnc_backend must be cpp or python" in result.stderr
    assert not capture.exists()


def test_hydra_cli_and_nodes_share_explicit_config_directory(tmp_path: Path, runner_env) -> None:
    env, capture = runner_env
    config = tmp_path / "working_config"
    shutil.copytree(REPO / "config", config)
    env["FSS_CONFIG_DIR"] = str(config)
    result = subprocess.run(
        [sys.executable, str(REPO / "scripts/run_experiment.py"),
         "system.use_sim=true", "system.gnc_backend=python",
         f"hydra.run.dir={tmp_path / 'hydra'}"],
        cwd=tmp_path, env=env, capture_output=True, text=True, timeout=20,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    invocation = json.loads(capture.read_text(encoding="utf-8"))
    assert invocation["config_dir"] == str(config)
    assert "gnc_backend:=python" in invocation["args"]
