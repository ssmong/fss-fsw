"""Linux 셸 경계에서 IMU 실행기의 드라이버 소유권과 권한 검사를 확인한다."""

import os
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
pytestmark = pytest.mark.skipif(os.name == "nt", reason="Linux Bash 실행기 검사")


@pytest.mark.parametrize("serial_permission", [True, False])
def test_imu_launcher_runs_only_raw_driver(tmp_path, serial_permission):
    capture = tmp_path / "invocation.txt"
    ros2 = tmp_path / "ros2"
    ros2.write_text('#!/bin/bash\nprintf "%s\\n" "$@" > "$FSS_TEST_CAPTURE"\n', encoding="utf-8")
    ros2.chmod(0o755)
    shell_env = tmp_path / "shell_env.sh"
    groups = "dialout" if serial_permission else "users"
    shell_env.write_text(
        f'source() {{ :; }}\nid() {{ printf "%s\\n" "{groups}"; }}\n',
        encoding="utf-8",
    )
    env = {
        **os.environ,
        "BASH_ENV": str(shell_env),
        "PATH": f"{tmp_path}{os.pathsep}{os.environ['PATH']}",
        "FSS_TEST_CAPTURE": str(capture),
    }
    result = subprocess.run(
        ["bash", str(REPO / "scripts/launch/start_imu.sh")],
        env=env, capture_output=True, text=True, encoding="utf-8", timeout=10,
    )
    if serial_permission:
        assert result.returncode == 0, result.stderr
        assert capture.read_text().splitlines() == [
            "launch", "xsens_mti_ros2_driver", "xsens_mti_node.launch.py",
        ]
    else:
        assert result.returncode == 1
        assert not capture.exists()
