"""Compiled manager/SimHal valve interoperability with the retained dry-run arm."""

import signal
import subprocess
import time
from pathlib import Path

import pytest
import yaml

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("fss_interfaces.msg")

from ament_index_python.packages import get_package_prefix  # noqa: E402
from fss_arm.arm_driver_node import ArmDriverNode  # noqa: E402
from fss_arm.commands import JOINT_NAMES  # noqa: E402
from fss_common.hydra_bridge import hydra_to_ros2_params, load_hydra_config  # noqa: E402
from fss_interfaces.msg import SpacecraftState, SystemHealth, SystemMode, ValveStatus  # noqa: E402
from fss_interfaces.srv import MainValveControl, SetSystemMode  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_srvs.srv import SetBool  # noqa: E402

REPO = Path(__file__).resolve().parents[2]


@pytest.mark.skipif(not Path("/proc").is_dir(), reason="Linux compiled ROS2 integration test")
def test_native_manager_valve_and_arm_sequence(tmp_path: Path) -> None:
    """IDLE→PRESSURIZING→READY→ACTIVE→arm motion→E-STOP through real ROS interfaces."""
    config = load_hydra_config(REPO / "config", overrides=["arm_driver_node.dry_run=true"])
    cfg = {
        name: {"ros__parameters": hydra_to_ros2_params(config, name)}
        for name in ("manager_node", "valve_driver_node", "arm_driver_node")
    }
    cfg["valve_driver_node"]["ros__parameters"].update(
        use_sim=True, use_thrusters=False, thrust_map_file="",
    )
    path = tmp_path / "system.yaml"
    path.write_text(yaml.safe_dump(cfg), encoding="utf-8")
    rclpy.init(args=["--ros-args", "--params-file", str(path)])
    arm = ArmDriverNode()
    helper = Node("native_arm_system_test_client")
    executor = SingleThreadedExecutor()
    executor.add_node(arm)
    executor.add_node(helper)
    nav = helper.create_publisher(SpacecraftState, "/navigation/state", 10)
    vicon = helper.create_publisher(SpacecraftState, "/sensors/vicon_state", 10)
    command = helper.create_publisher(JointState, "/arm/velocity_command", 1)
    health, statuses, states, modes = [], [], [], []
    helper.create_subscription(SystemHealth, "/system/health", health.append, 10)
    helper.create_subscription(ValveStatus, "/actuators/valve_status", statuses.append, 10)
    helper.create_subscription(JointState, "/arm/joint_states", states.append, 10)
    helper.create_subscription(SystemMode, "/system/mode", modes.append, 10)
    mode_client = helper.create_client(SetSystemMode, "/system/set_mode")
    valve_client = helper.create_client(MainValveControl, "/actuators/main_valve")
    arm_client = helper.create_client(SetBool, "/arm/set_enabled")
    processes, logs = [], []

    def sensors() -> None:
        state = SpacecraftState()
        state.header.stamp = helper.get_clock().now().to_msg()
        state.std_x, state.std_y, state.std_theta = 0.001, 0.001, 0.002
        nav.publish(state)
        vicon.publish(state)

    def velocity() -> None:
        msg = JointState()
        msg.header.stamp = helper.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.velocity = [0.05, 0.0, 0.0]
        command.publish(msg)

    helper.create_timer(0.01, sensors)
    helper.create_timer(0.01, velocity)

    def wait_for(predicate, description: str) -> None:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            for process, log_path in processes:
                assert process.poll() is None, log_path.read_text(encoding="utf-8")
            if predicate():
                return
            executor.spin_once(timeout_sec=0.005)
        pytest.fail(f"Timed out waiting for {description}")

    def call(client, request):
        assert client.wait_for_service(timeout_sec=5.0)
        future = client.call_async(request)
        executor.spin_until_future_complete(future, timeout_sec=2.0)
        response = future.result()
        assert response is not None and response.success, getattr(response, "message", "no response")
        return response

    def mode(value: int) -> None:
        request = SetSystemMode.Request()
        request.requested_mode = value
        call(mode_client, request)

    try:
        for package, executable in (
            ("fss_manager", "manager_node"), ("fss_actuators", "valve_driver_node"),
        ):
            binary = Path(get_package_prefix(package)) / "lib" / package / executable
            with binary.open("rb") as stream:
                assert stream.read(4) == b"\x7fELF", f"{binary} must be compiled"
            log_path = tmp_path / f"{executable}.log"
            log = log_path.open("w", encoding="utf-8")
            logs.append(log)
            process = subprocess.Popen(
                [str(binary), "--ros-args", "--params-file", str(path)],
                stdout=log, stderr=subprocess.STDOUT,
            )
            processes.append((process, log_path))

        wait_for(lambda: health and health[-1].navigation_ok and health[-1].actuators_ok
                 and not health[-1].navigation_degraded, "healthy native subsystems")
        mode(SystemMode.PRESSURIZING)
        opening = MainValveControl.Request()
        opening.open = True
        call(valve_client, opening)
        wait_for(lambda: health[-1].main_valve_open, "manager observing the open main valve")
        mode(SystemMode.READY)
        mode(SystemMode.ACTIVE)
        wait_for(lambda: arm.gate.active, "arm receiving native ACTIVE mode")
        request = SetBool.Request()
        request.data = True
        call(arm_client, request)
        wait_for(lambda: arm.enabled and not arm.gate.fault and states and statuses
                 and states[-1].velocity[0] > 0.0 and statuses[-1].main_valve_open,
                 "arm motion with the native main valve open")
        assert all(value == 0.0 for value in statuses[-1].applied_values)

        mode(SystemMode.EMERGENCY_STOP)
        wait_for(lambda: modes and modes[-1].mode == SystemMode.EMERGENCY_STOP
                 and not arm.enabled and "left ACTIVE" in arm.gate.fault
                 and not statuses[-1].main_valve_open
                 and all(value == 0.0 for value in statuses[-1].applied_values),
                 "native E-stop stopping the arm and closing the main valve")
    finally:
        for process, _ in processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
        for process, _ in processes:
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3.0)
        for log in logs:
            log.close()
        arm.close()
        executor.shutdown()
        arm.destroy_node()
        helper.destroy_node()
        rclpy.shutdown()
    for process, log_path in processes:
        assert process.returncode == 0, log_path.read_text(encoding="utf-8")
