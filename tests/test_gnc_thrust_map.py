"""Python 연구용 GNC → 실제 C++ 밸브 드라이버 → Python 플랜트 연결.

실행: bindings=ON 빌드 후 install/setup.bash를 불러와 pytest로 실행한다.
드라이버는 모의 HAL을 사용하며 실제 PLC와 연결하지 않는다.
"""

import importlib.util
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
import yaml
from fss_common.paths import config_file
from fss_common.valve_thrust_map import ValveThrustMap, load_valve_thrust_map
from fss_gnc._gnc_core import QPAllocator
from fss_sim.valve_model import ValveModel

pytest.importorskip("rclpy")

import rclpy
from ament_index_python.packages import get_package_prefix
from fss_interfaces.msg import SystemMode, ThrustCommands, ValveCommands, ValveStatus
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor

pytestmark = pytest.mark.skipif(not Path("/proc").is_dir(), reason="Linux ROS2 runtime test")


@pytest.fixture
def native_valve_driver(tmp_path, monkeypatch):
    """Return actual driver telemetry for research GNC thrust commands."""
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    context = Context()
    rclpy.init(context=context, domain_id=186)
    observer = rclpy.create_node("gnc_map_native_driver_test", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(observer)
    applied, statuses = [], []
    subscriptions = [
        observer.create_subscription(ValveCommands, "/actuators/valve_commands", applied.append, 10),
        observer.create_subscription(ValveStatus, "/actuators/valve_status", statuses.append, 10),
    ]
    mode_pub = observer.create_publisher(SystemMode, "/system/mode", 10)
    thrust_pub = observer.create_publisher(ThrustCommands, "/actuators/thrust_commands", 10)
    log_path = tmp_path / "native-valve.log"
    log = log_path.open("w", encoding="utf-8")
    process = None

    def wait_for(predicate, publish=None):
        deadline = time.monotonic() + 5
        next_publish = 0.0
        while time.monotonic() < deadline:
            assert process.poll() is None, log_path.read_text(encoding="utf-8")
            if predicate():
                return
            if publish is not None and time.monotonic() >= next_publish:
                publish()
                next_publish = time.monotonic() + 0.05
            executor.spin_once(timeout_sec=0.01)
        pytest.fail(f"Native driver did not return the expected output:\n{log_path.read_text()}")

    def start(map_filename):
        nonlocal process
        binary = Path(get_package_prefix("fss_actuators")) / "lib/fss_actuators/valve_driver_node"
        with binary.open("rb") as stream:
            assert stream.read(4) == b"\x7fELF", "valve_driver_node must be a native executable"
        parameters = tmp_path / "native-valve.yaml"
        parameters.write_text(yaml.safe_dump({"valve_driver_node": {"ros__parameters": {
            "use_sim": True, "use_sim_time": False, "use_thrusters": True,
            "thrust_map_file": str(map_filename), "max_thrust_per_nozzle": 2.0,
            "plc": {"ip": "127.0.0.1", "port": 502, "timeout_sec": 0.1,
                    "valve_register_start": 100, "valve_count": 8,
                    "main_valve_register": 99, "value_range": [0, 4000],
                    "deadband_minimum": 0, "control_rate_hz": 20.0},
        }}}), encoding="utf-8")
        environment = dict(os.environ, ROS_DOMAIN_ID="186")
        process = subprocess.Popen(
            [str(binary), "--ros-args", "--params-file", str(parameters)],
            env=environment, stdout=log, stderr=subprocess.STDOUT,
        )
        wait_for(lambda: mode_pub.get_subscription_count() and thrust_pub.get_subscription_count()
                 and statuses)

        def apply(command):
            received = (len(applied), len(statuses))
            thrust = ThrustCommands()
            thrust.header.stamp = observer.get_clock().now().to_msg()
            thrust.thrust_n = command.thrust_n
            thrust.enabled = command.enabled
            active = SystemMode()
            active.mode = SystemMode.ACTIVE
            has_thrust = any(command.thrust_n)

            def publish():
                mode_pub.publish(active)
                thrust_pub.publish(thrust)

            wait_for(lambda: len(applied) > received[0] and len(statuses) > received[1]
                     and bool(any(applied[-1].duty_cycles)) == has_thrust
                     and bool(any(statuses[-1].applied_values)) == has_thrust
                     and bool(any(statuses[-1].commanded_duty_cycles)) == has_thrust, publish)
            return applied[-1], statuses[-1]

        return apply

    try:
        yield start
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        log.close()
        for subscription in subscriptions:
            observer.destroy_subscription(subscription)
        executor.remove_node(observer)
        executor.shutdown()
        observer.destroy_node()
        rclpy.shutdown(context=context)


@pytest.mark.parametrize("map_kind", [None, "linear", "nonlinear"])
def test_gnc_thrust_reaches_plant_through_driver(monkeypatch, native_valve_driver, tmp_path, map_kind):
    use_map = map_kind is not None
    def message():
        return SimpleNamespace(header=SimpleNamespace())

    fake_ros_modules = {
        "rclpy": {},
        "rclpy.node": {"Node": object},
        "rclpy.parameter": {"Parameter": object},
        "fss_interfaces.msg": {
            "SpacecraftState": object,
            "TrajectoryReference": message,
            "ThrustCommands": message,
        },
        "fss_interfaces.srv": {"SetController": object, "SetGuidanceMode": object},
    }
    path = Path(__file__).resolve().parents[1] / "src/gnc/fss_gnc/fss_gnc/gnc_node.py"
    spec = importlib.util.spec_from_file_location("gnc_under_test", path)
    module = importlib.util.module_from_spec(spec)
    # Only the research node's ROS plumbing is replaced; the native driver uses real ROS below.
    with monkeypatch.context() as isolated:
        for name, values in fake_ros_modules.items():
            fake_module = ModuleType(name)
            fake_module.__dict__.update(values)
            isolated.setitem(sys.modules, name, fake_module)
        spec.loader.exec_module(module)
    node = object.__new__(module.GNCNode)
    thrust_map = (
        load_valve_thrust_map("calibration/valve_thrust_measured.yaml") if use_map else None
    )
    if map_kind == "nonlinear":
        # 실측 맵이 선형이어도 비선형 역변환과 밸브별 최대 추력 처리를 계속 검증한다.
        thrust_map = ValveThrustMap(
            [0.0, 0.5, 1.0], [[0.0, 0.1, 0.45 + 0.01 * i] for i in range(8)]
        )
        map_filename = tmp_path / "nonlinear.yaml"
        map_filename.write_text(yaml.safe_dump({
            "duty": [0.0, 0.5, 1.0],
            "thrust_n": [[0.0, 0.1, 0.45 + 0.01 * i] for i in range(8)],
        }), encoding="utf-8")
    else:
        map_filename = config_file("calibration", "valve_thrust_measured.yaml") if use_map else ""
    apply_native = native_valve_driver(map_filename)
    node._allocator = Mock(wraps=QPAllocator())
    monkeypatch.setattr(module, "load_valve_thrust_map", lambda filename: thrust_map)
    node.get_parameter = lambda name: SimpleNamespace(
        value="unused" if name.endswith("thrust_map_file") else 2.0
    )
    node.get_logger = lambda: Mock()
    node._setup_allocator_B()
    allocation_b = np.array(node._allocator.set_B_matrix.call_args.args[0]).reshape(3, 8)
    node._state_age_limit_ns = 100_000_000
    node._latest_state_ns = 0
    node._latest_state = SimpleNamespace(
        x=0,
        y=0,
        theta=0,
        vx=0,
        vy=0,
        omega=0,
        header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0)),
    )
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=0, to_msg=lambda: None)
    )
    node._t0, node._dt, node._max_force = None, 0.05, 2.0
    node._guidance = SimpleNamespace(
        compute=lambda *args: dict.fromkeys(
            ("x_ref", "y_ref", "theta_ref", "vx_ref", "vy_ref", "omega_ref"), 0.0
        )
    )
    node._trans_ctrl = SimpleNamespace(compute=lambda *args: SimpleNamespace(fx=0.7, fy=-0.4))
    node._rot_ctrl = SimpleNamespace(compute=lambda *args: 0.03)
    node._cmd_pub, node._ref_pub = Mock(), Mock()
    node._control_loop()
    command = node._cmd_pub.publish.call_args.args[0]
    assert command.enabled == [True] * 8
    allocated = node._allocator.allocate(0.7, -0.4, 0.03).duty_list()
    applied, status = apply_native(command)
    plant = ValveModel(
        model_mode="mapped" if use_map else "ideal", thrust_map=thrust_map,
        max_thrust_per_valve=2.0,
    )
    # Native inverse mapping must recover the planned thrust before PLC quantization.
    requested_thrust = plant.step(status.commanded_duty_cycles, 0.01)
    thrust = plant.step(applied.duty_cycles, 0.01)
    geometry = np.array(
        [
            [1, 1, -1, -1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 1, -1, -1],
            [-0.05, 0.05, 0.05, -0.05, 0.05, -0.05, -0.05, 0.05],
        ]
    )
    scale = thrust_map.max_thrust_n if use_map else 2.0
    assert allocation_b == pytest.approx(geometry * scale)
    assert command.thrust_n == pytest.approx(np.array(allocated) * scale)
    assert requested_thrust == pytest.approx(command.thrust_n, abs=1e-12)
    assert geometry @ requested_thrust == pytest.approx(allocation_b @ allocated, abs=1e-12)
    # The native driver's applied topic carries actual uint16 PLC registers / 4000.
    expected_registers = np.floor(np.array(status.commanded_duty_cycles) * 4000)
    assert status.applied_values == pytest.approx(expected_registers)
    assert applied.duty_cycles == pytest.approx(expected_registers / 4000, abs=1e-12)
    if use_map:
        assert thrust == pytest.approx(status.estimated_thrust, abs=1e-12)
    maximum_slope = (
        2 * (thrust_map.max_thrust_n - 0.1) if map_kind == "nonlinear"
        else np.full(8, 0.5 if use_map else 2.0)
    )
    thrust_error_limit = maximum_slope / 4000
    assert np.all(thrust <= np.array(command.thrust_n) + 1e-12)
    assert np.all(np.abs(thrust - command.thrust_n) <= thrust_error_limit + 1e-12)
    assert np.all(np.abs(geometry @ thrust - allocation_b @ allocated)
                  <= np.abs(geometry) @ thrust_error_limit + 1e-12)
    if map_kind == "nonlinear":
        assert not np.allclose(status.commanded_duty_cycles, allocated)
    else:
        assert status.commanded_duty_cycles == pytest.approx(allocated)
    node._publish_zero_command()
    zero = node._cmd_pub.publish.call_args.args[0]
    assert zero.thrust_n == [0.0] * 8
    applied_zero, status_zero = apply_native(zero)
    assert list(applied_zero.duty_cycles) == [0.0] * 8
    assert list(status_zero.applied_values) == [0.0] * 8
    assert plant.step(applied_zero.duty_cycles, 0.01).tolist() == [0.0] * 8
