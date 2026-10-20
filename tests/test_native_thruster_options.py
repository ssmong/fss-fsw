"""Installed native valve driver + retained SIL plant: option gates and read-only settings."""

import signal
import subprocess
import time
from pathlib import Path

import numpy as np
import pytest
import yaml
from omegaconf import OmegaConf

pytest.importorskip("rclpy")

import rclpy
from ament_index_python.packages import get_package_prefix
from fss_interfaces.msg import SystemMode, ThrustCommands, ValveCommands, ValveStatus
from fss_interfaces.srv import MainValveControl
from fss_sim.plant_model_node import PlantModelNode
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

REPO = Path(__file__).resolve().parents[1]


@pytest.mark.skipif(not Path("/proc").is_dir(), reason="Linux ROS2 runtime integration test")
@pytest.mark.parametrize("use_thrusters", [False, True])
@pytest.mark.parametrize("mode", [None, SystemMode.ACTIVE])
def test_native_thruster_option_and_main_valve(tmp_path, monkeypatch, use_thrusters, mode):
    monkeypatch.setenv("ROS_DOMAIN_ID", "181")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    cfg = OmegaConf.create({
        name: OmegaConf.load(REPO / "config" / path)
        for name, path in (
            ("vehicle", "vehicle/mass_properties.yaml"),
            ("ekf_node", "gnc/navigator/ekf.yaml"),
            ("valve_driver_node", "hardware/plc.yaml"),
            ("plant_model_node", "system/sim.yaml"),
        )
    })
    cfg.valve_thrust_map_file = ""
    cfg.plant_model_node = OmegaConf.merge(
        cfg.plant_model_node, OmegaConf.load(REPO / "config/system/sim_valve/ideal.yaml")
    )
    params = {
        name: {"ros__parameters": OmegaConf.to_container(cfg[name], resolve=True)}
        for name in ("valve_driver_node", "plant_model_node")
    }
    for values in params.values():
        values["ros__parameters"]["use_thrusters"] = use_thrusters
    params["valve_driver_node"]["ros__parameters"]["use_sim"] = True
    params["plant_model_node"]["ros__parameters"]["sim_rate_hz"] = 100.0
    path = tmp_path / "parameters.yaml"
    path.write_text(yaml.safe_dump(params), encoding="utf-8")
    binary = Path(get_package_prefix("fss_actuators")) / "lib/fss_actuators/valve_driver_node"
    with binary.open("rb") as stream:
        assert stream.read(4) == b"\x7fELF", "The option test must run the native driver"

    rclpy.init(args=["--ros-args", "--params-file", str(path)])
    nodes = []
    executor = SingleThreadedExecutor()
    process = None
    log_path = tmp_path / "native_valve.log"

    def output():
        return log_path.read_text(encoding="utf-8", errors="replace")

    def spin_for(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            assert process.poll() is None, output()
            executor.spin_once(timeout_sec=0.01)

    def wait_for(predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            spin_for(0.02)
        assert predicate(), output()

    try:
        with log_path.open("w", encoding="utf-8") as log:
            process = subprocess.Popen(
                [str(binary), "--ros-args", "--params-file", str(path)],
                stdout=log, stderr=subprocess.STDOUT,
            )
            plant = PlantModelNode()
            helper = Node("native_thruster_option_test")
            nodes.extend([plant, helper])
            for node in nodes:
                executor.add_node(node)
            initial = plant._dynamics.state.copy()
            received, applied = [], []
            subscriptions = [
                helper.create_subscription(ValveStatus, "/actuators/valve_status", received.append, 10),
                helper.create_subscription(ValveCommands, "/actuators/valve_commands", applied.append, 10),
            ]
            publisher = helper.create_publisher(ThrustCommands, "/actuators/thrust_commands", 10)
            mode_publisher = helper.create_publisher(SystemMode, "/system/mode", 10)
            set_parameters = helper.create_client(SetParameters, "/valve_driver_node/set_parameters")
            get_parameters = helper.create_client(GetParameters, "/valve_driver_node/get_parameters")
            client = helper.create_client(MainValveControl, "/actuators/main_valve")
            wait_for(lambda: publisher.get_subscription_count() >= 1
                and mode_publisher.get_subscription_count() >= 2
                and set_parameters.service_is_ready() and get_parameters.service_is_ready()
                and client.service_is_ready())
            if mode is not None:
                message = SystemMode()
                message.mode = mode
                mode_publisher.publish(message)
                spin_for(0.05)
            command = ThrustCommands()
            command.thrust_n = [0.8 * cfg.vehicle.max_thrust_per_nozzle] + [0.0] * 7
            command.enabled = [True] * 8
            timer = helper.create_timer(0.01, lambda: publisher.publish(command))

            request = MainValveControl.Request()
            request.open = True
            future = client.call_async(request)
            wait_for(future.done)
            assert future.result().success, future.result().message
            spin_for(0.6)
            assert received and received[-1].main_valve_open
            assert received[-1].commanded_duty_cycles == pytest.approx([0.8] + [0.0] * 7)
            assert applied
            if use_thrusters and mode == SystemMode.ACTIVE:
                assert applied[-1].duty_cycles[0] == pytest.approx(0.8)
                assert np.linalg.norm(plant._last_wrench) > 0
                assert np.linalg.norm(plant._dynamics.state - initial) > 1e-6
                assert received[-1].applied_values[0] == 3200.0
            else:
                assert list(applied[-1].duty_cycles) == [0.0] * 8
                np.testing.assert_array_equal(received[-1].applied_values, np.zeros(8))
                np.testing.assert_array_equal(plant._last_wrench, np.zeros(3))
                np.testing.assert_allclose(plant._dynamics.state, initial, atol=1e-14, rtol=0)

            setting = SetParameters.Request()
            setting.parameters = [Parameter("use_thrusters", value=not use_thrusters).to_parameter_msg()]
            future = set_parameters.call_async(setting)
            wait_for(future.done)
            assert not future.result().results[0].successful
            reading = GetParameters.Request()
            reading.names = ["use_thrusters"]
            future = get_parameters.call_async(reading)
            wait_for(future.done)
            assert future.result().values[0].bool_value == use_thrusters
            result = plant.set_parameters([Parameter("use_thrusters", value=not use_thrusters)])
            assert not result[0].successful
            assert plant.get_parameter("use_thrusters").value == use_thrusters
            assert timer and subscriptions
    finally:
        executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        rclpy.shutdown()
    assert process.returncode == 0, output()
