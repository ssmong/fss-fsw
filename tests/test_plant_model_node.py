"""실제 ROS 메시지로 SIL의 개별 밸브와 시스템 모드 차단을 검사한다."""

from unittest.mock import Mock

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("fss_interfaces.msg")

from fss_interfaces.msg import SystemMode, ValveCommands
from fss_sim.plant_model_node import PlantModelNode


def plant():
    node = PlantModelNode.__new__(PlantModelNode)
    node._use_thrusters = True
    node._latest_mode = None
    node._last_duty = [0.0] * 8
    node._valve_model = Mock()
    return node


def command():
    msg = ValveCommands()
    msg.duty_cycles = [0.8] * 8
    msg.enabled = [True] * 8
    return msg


@pytest.mark.parametrize(
    "mode", [None, SystemMode.IDLE, SystemMode.SAFE_HOLD, SystemMode.EMERGENCY_STOP],
)
def test_thrust_is_blocked_outside_active(mode):
    node = plant()
    if mode is not None:
        msg = SystemMode()
        msg.mode = mode
        node._on_mode(msg)
    node._on_valve_cmd(command())
    assert node._last_duty == [0.0] * 8


def test_active_respects_each_valve_enable():
    node = plant()
    mode = SystemMode()
    mode.mode = SystemMode.ACTIVE
    node._on_mode(mode)
    msg = command()
    msg.enabled = [True, False] * 4
    node._on_valve_cmd(msg)
    assert node._last_duty == [0.8, 0.0] * 4


@pytest.mark.parametrize("mode", [SystemMode.IDLE, SystemMode.SAFE_HOLD, SystemMode.EMERGENCY_STOP])
def test_leaving_active_clears_latched_thrust_without_another_command(mode):
    node = plant()
    msg = SystemMode()
    msg.mode = SystemMode.ACTIVE
    node._on_mode(msg)
    node._on_valve_cmd(command())
    assert node._last_duty == [0.8] * 8
    msg.mode = mode
    node._on_mode(msg)
    assert node._last_duty == [0.0] * 8
    node._valve_model.reset.assert_called_once()
