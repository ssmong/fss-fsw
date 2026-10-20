"""실제 SIL 런치에서 팔·추력기 조합과 하위 노드 파라미터 전달을 검사한다."""

import os
import signal
import subprocess
import time
from pathlib import Path

import pytest

pytest.importorskip("rclpy")

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node

REPO = Path(__file__).resolve().parents[1]


@pytest.fixture(autouse=True)
def isolated_ros_domain(monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "179")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")


@pytest.mark.parametrize(
    ("launch_file", "use_arm", "use_thrusters", "use_gnc"),
    [
        ("fss_full.launch.py", False, False, True),
        ("fss_full.launch.py", True, False, True),
        ("fss_full.launch.py", False, True, True),
        ("fss_full.launch.py", True, True, True),
        ("sim.launch.py", True, False, True),
        ("fss_full.launch.py", None, None, True),
        ("sim.launch.py", None, None, True),
        ("fss_full.launch.py", True, True, False),
    ],
)
def test_actual_launch_selection(
    tmp_path: Path,
    launch_file: str,
    use_arm: bool | None,
    use_thrusters: bool | None,
    use_gnc: bool,
    gnc_backend: str = "cpp",
) -> None:
    """추력 OFF는 use_gnc=true도 차단하고 팔과 메인밸브 노드는 독립 선택한다."""
    env = dict(os.environ)
    env["FSS_HYDRA_OVERRIDES"] = (
        "arm_driver_node.dry_run=true arm_driver_node.command_mode=velocity "
        "gnc_node.translational.kp=8.0"
    )
    command = [
        "ros2",
        "launch",
        "fss_bringup",
        launch_file,
        f"gnc_backend:={gnc_backend}",
    ]
    if launch_file == "fss_full.launch.py":
        command.append("use_sim:=true")
    if use_arm is not None:
        command.append(f"use_arm:={str(use_arm).lower()}")
    if use_thrusters is not None:
        command.append(f"use_thrusters:={str(use_thrusters).lower()}")
    if use_arm is not None or not use_gnc:
        command.append(f"use_gnc:={str(use_gnc).lower()}")
    use_arm = False if use_arm is None else use_arm
    use_thrusters = True if use_thrusters is None else use_thrusters
    gnc_expected = use_thrusters and use_gnc
    rclpy.init()
    helper = Node("launch_option_test")
    log_path = tmp_path / "launch.log"
    process = None
    try:
        with log_path.open("w", encoding="utf-8") as log:
            process = subprocess.Popen(
                command,
                cwd=REPO,
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            expected = {"plant_model_node", "ekf_node", "manager_node", "valve_driver_node"}
            if use_arm:
                expected.add("arm_driver_node")
            if gnc_expected:
                expected.add("gnc_node")
            deadline = time.monotonic() + 60
            while time.monotonic() < deadline:
                assert process.poll() is None, log_path.read_text(encoding="utf-8")
                names = set(helper.get_node_names())
                if expected <= names:
                    break
                rclpy.spin_once(helper, timeout_sec=0.1)
            assert expected <= names, log_path.read_text(encoding="utf-8")
            assert ("arm_driver_node" in names) == use_arm
            assert ("gnc_node" in names) == gnc_expected
            for node in sorted(expected):
                client = helper.create_client(GetParameters, f"/{node}/get_parameters")
                assert client.wait_for_service(timeout_sec=5)
                request = GetParameters.Request()
                request.names = ["use_sim_time"]
                if node in ("valve_driver_node", "plant_model_node"):
                    request.names.append("use_thrusters")
                if node == "valve_driver_node":
                    request.names.append("use_sim")
                if node == "gnc_node":
                    request.names.append("translational.kp")
                future = client.call_async(request)
                rclpy.spin_until_future_complete(helper, future, timeout_sec=5)
                assert future.result() is not None
                values = dict(zip(request.names, future.result().values, strict=True))
                assert values["use_sim_time"].bool_value == (
                    node not in ("plant_model_node", "arm_driver_node")
                )
                if "use_thrusters" in values:
                    assert values["use_thrusters"].bool_value == use_thrusters
                if "use_sim" in values:
                    assert values["use_sim"].bool_value
                if "translational.kp" in values:
                    assert values["translational.kp"].double_value == 8.0
                helper.destroy_client(client)
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
                pytest.fail(f"launch shutdown timeout: {log_path}")
        helper.destroy_node()
        rclpy.shutdown()
    output = log_path.read_text(encoding="utf-8")
    assert process.returncode == 0 and "Traceback" not in output, output
    if gnc_expected:
        expected_process = "gnc_node.py-" if gnc_backend == "python" else "gnc_node-"
        assert expected_process in output, output


def test_optional_python_research_backend_launches(tmp_path: Path) -> None:
    """Bindings-ON 빌드에서 명시적 Python 선택이 실제 노드까지 연결되는지 검사한다."""
    pytest.importorskip("fss_gnc._gnc_core", reason="optional bindings are disabled")
    test_actual_launch_selection(
        tmp_path, "sim.launch.py", False, True, True, gnc_backend="python",
    )
