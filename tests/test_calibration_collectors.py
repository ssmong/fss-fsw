"""장치 없이 실제 보정 수집 루프의 누락 측정·수신 중단을 검사한다."""

import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest


@pytest.fixture
def calibration_tools(monkeypatch):
    clock = SimpleNamespace(now=10.0, active_for=float("inf"), shutdown_at=float("inf"))
    nodes = []

    class Imu:
        def __init__(self):
            self.angular_velocity = SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.linear_acceleration = SimpleNamespace(x=0.0, y=0.0, z=9.80665)
            self.angular_velocity_covariance = [0.0] * 9
            self.linear_acceleration_covariance = [0.0] * 9

    class PoseStamped:
        def __init__(self):
            self.pose = SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0, z=0.1),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )

    class Node:
        def __init__(self, name):
            self.started_at = clock.now
            nodes.append(self)

        def create_subscription(self, message_type, topic, callback, qos):
            self.message_type = message_type
            self.callback = callback

        def destroy_node(self):
            pass

    def spin_once(node, *, timeout_sec):
        clock.now += 0.01
        if clock.now - node.started_at <= clock.active_for:
            node.callback(node.message_type())

    modules = {
        name: ModuleType(name)
        for name in (
            "rclpy", "rclpy.node", "rclpy.qos", "sensor_msgs", "sensor_msgs.msg",
            "geometry_msgs", "geometry_msgs.msg",
        )
    }
    modules["rclpy"].ok = lambda: clock.now < clock.shutdown_at
    modules["rclpy"].spin_once = spin_once
    modules["rclpy.node"].Node = Node
    modules["rclpy.qos"].qos_profile_sensor_data = object()
    modules["sensor_msgs.msg"].Imu = Imu
    modules["geometry_msgs.msg"].PoseStamped = PoseStamped
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)

    def load(name):
        path = Path(__file__).resolve().parents[1] / "scripts/calibration" / f"{name}.py"
        spec = importlib.util.spec_from_file_location(name, path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=lambda: clock.now))
        return module

    monkeypatch.setattr("builtins.input", lambda: "")
    return SimpleNamespace(
        imu=load("imu_static_check"), vicon=load("vicon_float_check"),
        Imu=Imu, clock=clock, nodes=nodes,
    )


@pytest.mark.parametrize("missing", ["angular_velocity_covariance", "linear_acceleration_covariance"])
def test_imu_collector_rejects_unavailable_required_measurement(calibration_tools, missing):
    module = calibration_tools.imu
    collector = module.IMUSampleCollector("/sensors/imu")
    message = calibration_tools.Imu()
    getattr(message, missing)[0] = -1.0
    collector._on_imu(message)
    assert collector.rejected_count == 1
    assert collector.gyro_samples == collector.accel_samples == []


@pytest.mark.parametrize("tool", ["imu", "vicon"])
@pytest.mark.parametrize("active_for", [1.0, 4.0])
def test_earlier_samples_cannot_hide_stream_stopping(calibration_tools, capsys, tool, active_for):
    runtime = calibration_tools
    runtime.clock.active_for = active_for
    module = getattr(runtime, tool)
    args = module.build_parser().parse_args(["--topic", "/test_sensor"])
    if tool == "vicon":
        args.air_off_duration = 5.0
    assert module.run_check(args) == 2
    assert "측정 종료 시" in capsys.readouterr().out

    collector = runtime.nodes[-1]
    rate = (
        module.sample_rate_hz(collector) if tool == "imu"
        else module.collection_stats(collector).sample_rate_hz
    )
    assert rate == pytest.approx(active_for * 20, abs=1.0)
    if active_for == 4.0:
        assert rate > args.min_rate  # 평균 수신률이 충분해도 종료 시 단절은 실패한다.


@pytest.mark.parametrize("tool", ["imu", "vicon"])
def test_continuous_stream_passes_collection_quality(calibration_tools, tool):
    module = getattr(calibration_tools, tool)
    assert module.run_check(module.build_parser().parse_args(["--topic", "/test_sensor"])) == 0


@pytest.mark.parametrize("tool", ["imu", "vicon"])
def test_ros_shutdown_cannot_pass_an_incomplete_collection(calibration_tools, tool):
    calibration_tools.clock.shutdown_at = 11.0
    module = getattr(calibration_tools, tool)
    with pytest.raises(RuntimeError, match="수집을 완료하지 못했다"):
        module.run_check(module.build_parser().parse_args(["--topic", "/test_sensor"]))
