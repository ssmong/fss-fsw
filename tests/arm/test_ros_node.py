"""실제 ROS2 DDS 토픽과 서비스로 dry-run 실행 및 정지를 검증한다."""

import json
import time
from pathlib import Path

import numpy as np
import pytest
import yaml

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("fss_interfaces.msg")

from fss_arm.arm_driver_node import ArmDriverNode  # noqa: E402
from fss_arm.commands import JOINT_NAMES  # noqa: E402
from fss_interfaces.msg import SystemMode  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_srvs.srv import SetBool  # noqa: E402

REPO = Path(__file__).resolve().parents[2]


def spin_for(executor: SingleThreadedExecutor, seconds: float) -> None:
    """지정 시간 동안 실제 ROS 콜백을 처리한다."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.005)


@pytest.fixture
def ros_arm(tmp_path: Path) -> tuple:
    """하드웨어를 열지 않는 노드와 외부 명령 발행자를 생성한다."""
    cfg = yaml.safe_load((REPO / "config/hardware/arm.yaml").read_text(encoding="utf-8"))
    cfg["initial_position_rad"] = [0.0, 0.0, 0.0]
    params = tmp_path / "params.yaml"
    params.write_text(
        yaml.safe_dump({"arm_driver_node": {"ros__parameters": cfg}}), encoding="utf-8"
    )
    rclpy.init(args=["--ros-args", "--params-file", str(params)])
    arm = ArmDriverNode()
    helper = Node("arm_test_client")
    executor = SingleThreadedExecutor()
    executor.add_node(arm)
    executor.add_node(helper)
    mode_pub = helper.create_publisher(SystemMode, "/system/mode", 1)
    command_pub = helper.create_publisher(JointState, "/arm/velocity_command", 1)
    client = helper.create_client(SetBool, "/arm/set_enabled")
    states = []
    helper.create_subscription(JointState, "/arm/joint_states", states.append, 10)

    def heartbeat() -> None:
        msg = SystemMode()
        msg.mode = SystemMode.ACTIVE
        msg.header.stamp = helper.get_clock().now().to_msg()
        mode_pub.publish(msg)

    timer = helper.create_timer(0.02, heartbeat)
    spin_for(executor, 0.15)
    assert client.wait_for_service(timeout_sec=1)
    try:
        yield arm, helper, executor, mode_pub, command_pub, client, states, timer
    finally:
        arm.close()
        executor.shutdown()
        arm.destroy_node()
        helper.destroy_node()
        rclpy.shutdown()


def enable(ros_arm: tuple) -> None:
    """외부 enable 서비스를 호출한다."""
    _, _, executor, _, _, client, _, _ = ros_arm
    request = SetBool.Request()
    request.data = True
    future = client.call_async(request)
    executor.spin_until_future_complete(future, timeout_sec=1)
    assert future.result().success, future.result().message


def velocity_message(helper: Node, values: list[float]) -> JointState:
    """ROS clock으로 timestamp를 붙인 관절속도 명령."""
    msg = JointState()
    msg.header.stamp = helper.get_clock().now().to_msg()
    msg.name = list(JOINT_NAMES)
    msg.velocity = values
    return msg


def test_streaming_feedback_and_timeout(ros_arm: tuple) -> None:
    """실제 메시지로 움직인 뒤 명령이 끊기면 fault와 정지 상태를 발행한다."""
    arm, helper, executor, _, command_pub, _, states, _ = ros_arm
    enable(ros_arm)
    timer = helper.create_timer(
        0.01, lambda: command_pub.publish(velocity_message(helper, [0.2, -0.1, 0.0]))
    )
    spin_for(executor, 0.15)
    assert any(s.velocity[0] > 0.1 and s.velocity[1] < 0 for s in states)
    timer.cancel()
    spin_for(executor, 0.15)
    assert not arm.enabled and "expired" in arm.gate.fault
    np.testing.assert_array_equal(arm.velocity, [0, 0, 0])


def test_mode_exit_stops_and_latches(ros_arm: tuple) -> None:
    """EMERGENCY_STOP 후 ACTIVE heartbeat가 돌아와도 자동 재시작하지 않는다."""
    arm, _, executor, mode_pub, _, _, _, timer = ros_arm
    enable(ros_arm)
    timer.cancel()
    mode = SystemMode()
    mode.mode = SystemMode.EMERGENCY_STOP
    mode_pub.publish(mode)
    spin_for(executor, 0.05)
    assert not arm.enabled and "left ACTIVE" in arm.gate.fault
    mode.mode = SystemMode.ACTIVE
    mode_pub.publish(mode)
    spin_for(executor, 0.03)
    assert not arm.enabled


@pytest.mark.parametrize("invalid", ["nan", "stale", "order", "speed"])
def test_bad_ros_command_stops(ros_arm: tuple, invalid: str) -> None:
    """잘못된 명령이 최신 명령을 대체하지 못하도록 정지한다."""
    arm, helper, executor, _, publisher, _, _, _ = ros_arm
    enable(ros_arm)
    msg = velocity_message(helper, [0.1, 0.0, 0.0])
    if invalid == "nan":
        msg.velocity[0] = float("nan")
    elif invalid == "stale":
        msg.header.stamp.sec -= 1
    elif invalid == "order":
        msg.name.reverse()
    else:
        msg.velocity[0] = 2.0
    publisher.publish(msg)
    spin_for(executor, 0.05)
    assert not arm.enabled and arm.gate.fault


def test_slow_hardware_read_cannot_send_expired_command(
    ros_arm: tuple, monkeypatch: pytest.MonkeyPatch
) -> None:
    """읽기 시작 시 유효했던 명령도 읽기 종료 때 만료되면 전송하지 않는다."""
    from types import SimpleNamespace

    arm, _, _, _, _, _, _, _ = ros_arm
    now = [time.monotonic()]
    arm.last_tick = now[0]
    arm.enabled = True
    arm.command = np.array([0.2, 0.0, 0.0])
    arm.gate.command_received = now[0]
    sent = []

    def read_state() -> tuple[np.ndarray, np.ndarray]:
        now[0] += 0.15
        return np.zeros(3), np.zeros(3)

    arm.bus = SimpleNamespace(
        read_state=read_state, write_velocity=sent.append, disable=lambda: None, current=np.zeros(3)
    )
    with monkeypatch.context() as patch:
        patch.setattr(time, "monotonic", lambda: now[0])
        arm._tick()
    arm.bus = None
    assert not sent and not arm.enabled
    assert "deadline" in arm.gate.fault


def test_source_timestamp_limits_remaining_command_lifetime(ros_arm: tuple) -> None:
    """전송 지연이 있는 명령에 수신 시점부터 timeout 전체를 다시 주지 않는다."""
    from rclpy.duration import Duration

    arm, helper, executor, _, publisher, _, _, _ = ros_arm
    enable(ros_arm)
    msg = velocity_message(helper, [0.1, 0.0, 0.0])
    msg.header.stamp = (helper.get_clock().now() - Duration(seconds=0.08)).to_msg()
    # enable 이후에 생성했지만 네트워크 지연된 명령이라는 상황을 구성한다.
    arm.last_stamp_ns = 0
    arm.enable_stamp_ns = 0
    publisher.publish(msg)
    spin_for(executor, 0.055)
    assert not arm.enabled and "expired" in arm.gate.fault


def test_read_overrun_is_reported_at_control_period(
    ros_arm: tuple, monkeypatch: pytest.MonkeyPatch
) -> None:
    """100 ms 정지 한계보다 짧은 12 ms 읽기도 100 Hz 예산 초과로 기록한다."""
    from types import SimpleNamespace

    arm, _, _, _, _, _, _, _ = ros_arm
    now = [time.monotonic()]
    arm.last_tick = now[0]
    arm.enabled = True
    arm.gate.command_received = now[0]

    def read_state() -> tuple[np.ndarray, np.ndarray]:
        now[0] += 0.012
        return np.zeros(3), np.zeros(3)

    arm.bus = SimpleNamespace(
        read_state=read_state,
        write_velocity=lambda raw: None,
        disable=lambda: None,
        current=np.zeros(3),
    )
    try:
        with monkeypatch.context() as patch:
            patch.setattr(time, "monotonic", lambda: now[0])
            arm._tick()
        assert arm.enabled and not arm.gate.fault
        assert arm.timing["cycle_overruns"] == 1
        assert arm.timing["last_read_ms"] == pytest.approx(12.0, abs=1e-4)
        assert arm.timing["max_cycle_ms"] == pytest.approx(12.0, abs=1e-4)
    finally:
        arm.bus = None


def test_slow_write_trips_and_retains_timing(
    ros_arm: tuple, monkeypatch: pytest.MonkeyPatch
) -> None:
    """쓰기 중 100 ms를 넘으면 정지를 요청하고 실패 주기의 통계를 남긴다."""
    from types import SimpleNamespace

    arm, _, _, _, _, _, _, _ = ros_arm
    now = [time.monotonic()]
    arm.last_tick = now[0]
    arm.enabled = True
    arm.gate.command_received = now[0]
    stopped = []

    def write(raw: tuple) -> None:
        now[0] += 0.15

    arm.bus = SimpleNamespace(
        current=np.zeros(3),
        read_state=lambda: (np.zeros(3), np.zeros(3)),
        write_velocity=write,
        disable=lambda: stopped.append(True),
    )
    try:
        with monkeypatch.context() as patch:
            patch.setattr(time, "monotonic", lambda: now[0])
            arm._tick()
        assert stopped and not arm.enabled and "during write" in arm.gate.fault
        assert arm.timing["last_write_ms"] == pytest.approx(150.0, abs=1e-4)
    finally:
        arm.bus = None


def test_timing_topic_contains_enabled_cycle_measurements(ros_arm: tuple) -> None:
    """실제 DDS 토픽에서 dry_run 구분과 실행 주기 통계를 읽을 수 있다."""
    from std_msgs.msg import String

    _, helper, executor, _, publisher, _, _, _ = ros_arm
    samples = []
    helper.create_subscription(
        String, "/arm/timing", lambda msg: samples.append(json.loads(msg.data)), 10
    )
    enable(ros_arm)
    timer = helper.create_timer(
        0.01, lambda: publisher.publish(velocity_message(helper, [0.05, 0.0, 0.0]))
    )
    spin_for(executor, 1.15)
    timer.cancel()
    assert samples and samples[-1]["dry_run"]
    assert samples[-1]["cycles"] > 0 and samples[-1]["target_period_ms"] == 10.0
    assert samples[-1]["last_read_ms"] == 0.0


@pytest.mark.parametrize(
    "overrides",
    [
        {"bus_watchdog_s": 0.029, "control_rate_hz": 40.0},
        {"bus_watchdog_s": float("nan")},
        {"bus_watchdog_s": 0.01},
        {"return_delay_us": 3},
        {"return_delay_us": -2},
    ],
)
def test_invalid_bus_timing_rejected_in_dry_run(tmp_path: Path, overrides: dict) -> None:
    """실제 적용 watchdog과 지연 설정은 dry_run에서도 기동 전에 검증한다."""
    cfg = yaml.safe_load((REPO / "config/hardware/arm.yaml").read_text(encoding="utf-8"))
    cfg.update(overrides)
    path = tmp_path / "invalid.yaml"
    path.write_text(yaml.safe_dump({"arm_driver_node": {"ros__parameters": cfg}}), encoding="utf-8")
    rclpy.init(args=["--ros-args", "--params-file", str(path)])
    try:
        with pytest.raises(ValueError):
            ArmDriverNode()
    finally:
        rclpy.shutdown()


def switch_mode(ros_arm: tuple, mode: str) -> object:
    """실제 ROS 서비스로 명령 모드를 전환한다."""
    from fss_arm_msgs.srv import SetCommandMode

    _, helper, executor, _, _, _, _, _ = ros_arm
    client = helper.create_client(SetCommandMode, "/arm/set_command_mode")
    assert client.wait_for_service(timeout_sec=1)
    future = client.call_async(SetCommandMode.Request(mode=mode))
    executor.spin_until_future_complete(future, timeout_sec=1)
    assert future.result() is not None
    return future.result()


@pytest.mark.parametrize("mode", ["position", "current"])
def test_new_command_modes_and_timeout(ros_arm: tuple, mode: str) -> None:
    """실제 DDS 위치·전류 입력과 모드 유지·명령 만료 정지를 확인한다."""
    from fss_arm_msgs.msg import JointCurrent

    arm, helper, executor, _, _, _, _, _ = ros_arm
    assert switch_mode(ros_arm, mode).success
    message_type = JointState if mode == "position" else JointCurrent
    publisher = helper.create_publisher(message_type, f"/arm/{mode}_command", 1)
    feedback = []
    helper.create_subscription(JointCurrent, "/arm/currents", feedback.append, 10)
    enable(ros_arm)

    def command() -> None:
        msg = message_type()
        msg.header.stamp = helper.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        setattr(msg, mode, [0.1, -0.1, 0.0])
        publisher.publish(msg)

    timer = helper.create_timer(0.01, command)
    spin_for(executor, 0.2)
    assert arm.enabled and arm.command_mode == mode
    if mode == "position":
        assert arm.position[0] > 0.02 and arm.position[1] < -0.02
    else:
        assert feedback and feedback[-1].current[0] > 0.09
        np.testing.assert_array_equal(arm.position, [0, 0, 0])
    assert not switch_mode(ros_arm, "velocity").success
    assert arm.enabled and arm.command_mode == mode
    timer.cancel()
    spin_for(executor, 0.15)
    assert not arm.enabled and "expired" in arm.gate.fault


@pytest.mark.parametrize(
    "mode,failure",
    [
        ("position", "limit"),
        ("current", "limit"),
        ("position", "wrong_mode"),
        ("current", "stale"),
        ("current", "nan"),
    ],
)
def test_new_mode_invalid_commands_stop(ros_arm: tuple, mode: str, failure: str) -> None:
    """각도·전류 한계와 잘못된 모드·시각·비유한 입력을 거부한다."""
    from fss_arm_msgs.msg import JointCurrent

    arm, helper, executor, _, velocity_pub, _, _, _ = ros_arm
    assert switch_mode(ros_arm, mode).success
    message_type = JointState if mode == "position" else JointCurrent
    publisher = helper.create_publisher(message_type, f"/arm/{mode}_command", 1)
    enable(ros_arm)
    if failure == "wrong_mode":
        velocity_pub.publish(velocity_message(helper, [0.1, 0.0, 0.0]))
    else:
        msg = message_type()
        msg.header.stamp = helper.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        values = [0.1, 0.0, 0.0]
        if failure == "limit":
            values[0] = 3.0
        elif failure == "nan":
            values[0] = float("nan")
        else:
            msg.header.stamp.sec -= 1
        setattr(msg, mode, values)
        publisher.publish(msg)
    spin_for(executor, 0.05)
    assert not arm.enabled and arm.gate.fault


def test_mode_switch_clears_command_and_preserves_position(ros_arm: tuple) -> None:
    """disable 후 모드 전환은 이전 값을 버리며 위치 enable은 현재 각도에서 시작한다."""
    arm, _, _, _, _, _, _, _ = ros_arm
    arm.position = np.array([0.1, -0.2, 0.3])
    arm.command = np.array([0.2, 0.2, 0.2])
    assert switch_mode(ros_arm, "position").success
    assert arm.gate.command_received == float("-inf")
    enable(ros_arm)
    np.testing.assert_allclose(arm.command, arm.position)
    arm._stop()
    assert switch_mode(ros_arm, "current").success
    np.testing.assert_array_equal(arm.command, [0, 0, 0])


def test_queued_command_from_before_enable_is_discarded(ros_arm: tuple) -> None:
    """enable 전 생성된 신선한 명령은 큐에서 늦게 도착해도 적용·fault 처리하지 않는다."""
    arm, helper, executor, _, publisher, _, _, _ = ros_arm
    queued = velocity_message(helper, [0.4, 0.0, 0.0])
    enable(ros_arm)
    publisher.publish(queued)
    spin_for(executor, 0.015)
    assert arm.enabled and not arm.gate.fault
    np.testing.assert_array_equal(arm.command, [0, 0, 0])
    publisher.publish(velocity_message(helper, [0.05, 0.0, 0.0]))
    spin_for(executor, 0.015)
    assert arm.command[0] == 0.05


def test_current_mode_measured_overspeed_stops(ros_arm: tuple) -> None:
    """전류 모드에서 명령 전류가 작아도 실제 속도가 한계를 넘으면 정지한다."""
    from types import SimpleNamespace

    arm, _, _, _, _, _, _, _ = ros_arm
    assert switch_mode(ros_arm, "current").success
    enable(ros_arm)
    stopped, sent = [], []
    arm.bus = SimpleNamespace(
        current=np.zeros(3),
        read_state=lambda: (np.zeros(3), np.array([2.0, 0.0, 0.0])),
        write_current=sent.append,
        disable=lambda: stopped.append(True),
    )
    try:
        arm._tick()
        assert stopped and not sent and not arm.enabled
        assert "measured velocity limit" in arm.gate.fault
    finally:
        arm.bus = None
