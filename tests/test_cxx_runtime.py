"""Built ROS2 C++ chain + Python SIL plant: real topics, services and E-stop.

Run after colcon build and source install/setup.bash. No physical devices are used.
The test deliberately fails if a default onboard executable is a Python wrapper.
"""

import math
import os
import re
import shlex
import signal
import subprocess
import time
from pathlib import Path

import pytest

pytest.importorskip("rclpy")

import rclpy
from ament_index_python.packages import get_package_prefix
from fss_interfaces.msg import (
    SpacecraftState, SystemHealth, SystemMode, ThrustCommands, ValveCommands, ValveStatus,
)
from fss_interfaces.srv import MainValveControl, SetSystemMode
from rclpy.node import Node

pytestmark = pytest.mark.skipif(not Path("/proc").is_dir(), reason="Linux ROS2 runtime tests")


def check_native_chain(tmp_path, monkeypatch, overrides=(), convergence=None):
    monkeypatch.setenv("ROS_DOMAIN_ID", "179")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    binaries = {}
    for package, names in {
        "fss_gnc": ["gnc_node"],
        "fss_navigation": ["ekf_node"],
        "fss_actuators": ["valve_driver_node", "reaction_wheel_driver_node"],
        "fss_manager": ["manager_node"],
        "fss_sensors": ["imu_bridge_node", "vicon_bridge_node"],
    }.items():
        for name in names:
            binary = Path(get_package_prefix(package)) / "lib" / package / name
            with binary.open("rb") as stream:
                assert stream.read(4) == b"\x7fELF", f"{binary} must be a compiled executable"
            binaries[name] = binary.resolve()

    env = dict(os.environ)
    env["FSS_HYDRA_OVERRIDES"] = shlex.join(overrides)
    log_path = tmp_path / "cxx_sil.log"
    rclpy.init()
    helper = Node("cxx_runtime_smoke")
    latest = {}
    counts = {}

    def receive(name, message):
        latest[name] = message
        counts[name] = counts.get(name, 0) + 1

    subscriptions = [
        helper.create_subscription(message_type, topic,
                                   lambda message, key=name: receive(key, message), 10)
        for name, message_type, topic in [
            ("nav", SpacecraftState, "/navigation/state"),
            ("truth", SpacecraftState, "/sim/true_state"),
            ("thrust", ThrustCommands, "/actuators/thrust_commands"),
            ("valves", ValveCommands, "/actuators/valve_commands"),
            ("status", ValveStatus, "/actuators/valve_status"),
            ("health", SystemHealth, "/system/health"),
            ("mode", SystemMode, "/system/mode"),
        ]
    ]
    mode_client = helper.create_client(SetSystemMode, "/system/set_mode")
    valve_client = helper.create_client(MainValveControl, "/actuators/main_valve")
    process = None

    def output():
        return log_path.read_text(encoding="utf-8", errors="replace")

    def wait_for(predicate, description, timeout=30):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            assert process.poll() is None, output()
            if predicate():
                return
            rclpy.spin_once(helper, timeout_sec=0.1)
        pytest.fail(f"Timed out waiting for {description}\n{output()}")

    def call(client, request):
        assert client.wait_for_service(timeout_sec=10), output()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(helper, future, timeout_sec=5)
        assert future.done() and future.result() is not None, output()
        return future.result()

    def transition(mode):
        # Readiness is refreshed on the manager's health timer, after service replies.
        deadline = time.monotonic() + 15
        request = SetSystemMode.Request()
        request.requested_mode = mode
        while time.monotonic() < deadline:
            response = call(mode_client, request)
            if response.success:
                assert response.actual_mode == mode
                return
            rclpy.spin_once(helper, timeout_sec=0.1)
        pytest.fail(f"Mode {mode} rejected: {response.message}\n{output()}")

    try:
        with log_path.open("w", encoding="utf-8") as log:
            process = subprocess.Popen(
                ["ros2", "launch", "fss_bringup", "fss_full.launch.py",
                 "use_sim:=true", "cpu_pin:=false", "gnc_rt_priority:=0", "actuator_rt_priority:=0"],
                cwd=tmp_path, env=env, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            wait_for(lambda: {"nav", "truth", "valves", "status", "health", "mode"} <= latest.keys(),
                     "the default chain and simulated sensor stream", timeout=60)
            launched = dict(re.findall(
                r"\[(gnc_node|ekf_node|valve_driver_node|manager_node)-\d+\].*"
                r"process started with pid \[(\d+)\]", output(),
            ))
            assert set(launched) == {"gnc_node", "ekf_node", "valve_driver_node", "manager_node"}, output()
            for name, pid in launched.items():
                assert Path(f"/proc/{pid}/exe").resolve() == binaries[name], output()

            assert latest["mode"].mode == SystemMode.IDLE
            assert not any(latest["valves"].duty_cycles)
            assert not latest["status"].main_valve_open
            wait_for(lambda: latest["health"].navigation_ok and latest["health"].actuators_ok,
                     "healthy navigation and simulated actuator feedback")
            for field in ("x", "y", "theta", "vx", "vy", "omega", "std_x", "std_y", "std_theta"):
                assert math.isfinite(getattr(latest["nav"], field))
            assert latest["nav"].header.stamp.sec or latest["nav"].header.stamp.nanosec

            transition(SystemMode.PRESSURIZING)
            open_request = MainValveControl.Request()
            open_request.open = True
            assert call(valve_client, open_request).success
            transition(SystemMode.READY)
            initial = (latest["truth"].x, latest["truth"].y, latest["truth"].theta)
            transition(SystemMode.ACTIVE)
            active_stamp = latest["truth"].header.stamp
            active_time = active_stamp.sec + active_stamp.nanosec * 1e-9
            wait_for(lambda: "thrust" in latest and any(latest["thrust"].thrust_n)
                     and any(latest["valves"].duty_cycles)
                     and any(latest["status"].applied_values), "GNC thrust reaching simulated valves")
            wait_for(lambda: math.dist(initial, (latest["truth"].x, latest["truth"].y,
                                                latest["truth"].theta)) > 1e-4,
                     "plant response to applied thrust")
            assert all(math.isfinite(value) and 0 <= value <= 1
                       for value in latest["valves"].duty_cycles)
            wait_for(lambda: counts["nav"] > 10, "continued navigation stream")

            if convergence is not None:
                target, duration, position_limit = convergence
                wait_for(lambda: latest["mode"].mode == SystemMode.ACTIVE, "ACTIVE publication")

                def truth_time():
                    stamp = latest["truth"].header.stamp
                    return stamp.sec + stamp.nanosec * 1e-9

                end_time = active_time + duration

                def completed():
                    assert latest["mode"].mode == SystemMode.ACTIVE, output()
                    return truth_time() >= end_time

                wait_for(completed, f"{duration}s of closed-loop simulated time", timeout=duration * 4 + 30)
                state = latest["truth"]
                position_error = math.hypot(state.x - target[0], state.y - target[1])
                heading_error = abs(math.remainder(state.theta - target[2], 2 * math.pi))
                print(f"Native SIL {duration}s: position={position_error:.6f}m heading={heading_error:.6f}rad")
                assert position_error < position_limit, (position_error, position_limit, output())
                assert heading_error < 0.1, (heading_error, output())

            transition(SystemMode.EMERGENCY_STOP)
            wait_for(lambda: latest["mode"].mode == SystemMode.EMERGENCY_STOP
                     and not any(latest["valves"].duty_cycles)
                     and not any(latest["status"].applied_values)
                     and not latest["status"].main_valve_open,
                     "E-stop zero outputs and closed main valve")
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        for subscription in subscriptions:
            helper.destroy_subscription(subscription)
        helper.destroy_node()
        rclpy.shutdown()
    assert process.returncode == 0, output()
    assert "Traceback" not in output(), output()


def test_default_cxx_chain_and_estop_with_python_plant(tmp_path, monkeypatch):
    check_native_chain(tmp_path, monkeypatch)


@pytest.mark.parametrize(
    ("valve_mode", "initial", "target", "duration", "position_limit"),
    [
        ("mapped", (0.5, 0.3, 0.2), (0.0, 0.0, 0.0), 30.0, 0.05),
        ("real", (0.5, 0.3, 0.2), (0.0, 0.0, 0.0), 45.0, 0.08),
        ("real", (0.0, 0.0, 0.0), (0.3, -0.2, 0.5), 30.0, 0.05),
    ],
    ids=["mapped-origin-30s", "physical-origin-45s", "physical-target-30s"],
)
def test_native_closed_loop_convergence(
    tmp_path, monkeypatch, valve_mode, initial, target, duration, position_limit,
):
    """Preserve the former Python EKF SIL scenarios using the actual C++ runtime."""
    overrides = [
        f"system/sim_valve={valve_mode}",
        "vehicle.mass=10.0", "vehicle.inertia_zz=0.5",
        "vehicle.max_force_body=5.0", "vehicle.max_torque_body=2.0",
        "gnc_node.translational.kp=10.0", "gnc_node.translational.ki=0.5",
        "gnc_node.translational.kd=12.0", "gnc_node.rotational.kp=5.0",
        "gnc_node.rotational.ki=0.1", "gnc_node.rotational.kd=2.0",
    ]
    if valve_mode == "mapped":
        overrides.append("valve_thrust_map_file=calibration/valve_thrust_measured.yaml")
    for axis, initial_value, target_value in zip(("x", "y", "theta"), initial, target, strict=True):
        overrides.extend([
            f"plant_model_node.initial_{axis}={initial_value}",
            f"gnc_node.guidance.target_{axis}={target_value}",
        ])
    check_native_chain(
        tmp_path, monkeypatch, overrides,
        convergence=(target, duration, position_limit),
    )
