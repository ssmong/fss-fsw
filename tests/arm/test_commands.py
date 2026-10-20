"""단위, signed 인코딩, 물리 한계와 명령 만료 검사."""

from dataclasses import replace

import numpy as np
import pytest
from fss_arm.commands import (
    VELOCITY_RAD_S_PER_LSB,
    CommandGate,
    JointMapping,
    signed32,
    watchdog_ticks,
)


@pytest.fixture
def mapping() -> JointMapping:
    """회전 부호와 외부 감속비가 다른 3축 fixture."""
    return JointMapping(
        (1, 2, 3), (1, -1, 1), (1, 2, 3), (2048, 0, 4096), (1, 1, 1), (-2, -2, -2), (2, 2, 2)
    )


def test_conversion_direction_gearing_and_zero(mapping: JointMapping) -> None:
    """관절속도와 출력축 속도 및 zero offset 관계를 검증한다."""
    step = VELOCITY_RAD_S_PER_LSB
    raw = mapping.encode_velocity([step * 10, step * 10, -step * 10])
    assert raw == (10, -20, -30)
    np.testing.assert_allclose(mapping.decode_velocity(raw), [step * 10, step * 10, -step * 10])
    np.testing.assert_allclose(mapping.decode_position([2048, 4096, 4096]), [0, -np.pi, 0])
    assert signed32(0xFFFFFFFF) == -1
    assert signed32(0x80000000) == -2147483648


def test_quantization_does_not_exceed_joint_speed_limit(mapping: JointMapping) -> None:
    """1 rad/s를 42 LSB로 올려 1 rad/s를 초과하지 않는다."""
    decoded = mapping.decode_velocity(mapping.encode_velocity([1, -1, 1]))
    assert np.all(np.abs(decoded) <= 1)
    assert mapping.encode_velocity([1, 0, 0])[0] == 41


@pytest.mark.parametrize(
    "value", [[float("nan"), 0, 0], [0, float("inf"), 0], [0, 0], [1.01, 0, 0]]
)
def test_invalid_velocity_rejected(mapping: JointMapping, value: list[float]) -> None:
    """비정상 입력을 잘라서 숨기지 않고 거부한다."""
    with pytest.raises(ValueError):
        mapping.encode_velocity(value)


def test_unknown_mapping_rejected(mapping: JointMapping) -> None:
    """미확정 부호/중복 ID/0 감속비는 사용할 수 없다."""
    for changes in ({"ids": (1, 1, 3)}, {"directions": (0, 1, 1)}, {"gear_ratios": (0, 1, 1)}):
        with pytest.raises(ValueError):
            replace(mapping, **changes)


def test_mode_command_expiry_and_fault_latch() -> None:
    """ACTIVE만 복구되어도 이전 명령이 다시 움직이지 않는다."""
    gate = CommandGate(0.5, 0.1)
    assert not gate.ready(1)
    gate.update_mode(True, 1)
    gate.command_received = 1
    assert gate.ready(1.05)
    assert not gate.ready(1.11)
    gate.update_mode(False, 1.12)
    gate.update_mode(True, 1.13)
    assert not gate.ready(1.13)
    gate.command_received = 1.13
    gate.fault = "serial error"
    assert not gate.ready(1.13)
    gate.fault = ""
    gate.command_received = 2
    assert not gate.ready(2)


def test_watchdog_units() -> None:
    """하드웨어 watchdog의 20 ms 단위를 검사한다."""
    assert watchdog_ticks(0.1) == 5
    assert watchdog_ticks(0.119) == 5
    with pytest.raises(ValueError):
        watchdog_ticks(0)


def test_position_roundtrip_and_quantized_limits(mapping: JointMapping) -> None:
    """위치 명령의 영점·부호·감속비와 한계 안쪽 양자화를 검사한다."""
    q = np.array([0.2, -0.4, 0.6])
    restored = mapping.decode_position(mapping.encode_position(q))
    np.testing.assert_allclose(restored, q, atol=0.001)
    for boundary in (mapping.lower_limits, mapping.upper_limits):
        mapping.check_position(mapping.decode_position(mapping.encode_position(boundary)))
    with pytest.raises(ValueError):
        mapping.encode_position([3, 0, 0])


def test_current_units_sign_and_no_gearing(mapping: JointMapping) -> None:
    """전류는 A→2.69 mA 단위이며 방향만 적용하고 감속비는 적용하지 않는다."""
    from fss_arm.commands import decode_current, encode_current, signed16

    values = [0.0269, 0.0269, -0.0269]
    raw = encode_current(mapping, values, [0.2] * 3)
    assert raw == (10, -10, -10)
    np.testing.assert_allclose(decode_current(mapping, raw), values)
    assert signed16(65535) == -1
    assert np.all(
        np.abs(decode_current(mapping, encode_current(mapping, [0.2] * 3, [0.2] * 3))) <= 0.2
    )
    with pytest.raises(ValueError):
        encode_current(mapping, [0.21, 0, 0], [0.2] * 3)


def test_position_profile_never_uses_unlimited_zero(mapping: JointMapping) -> None:
    """양자화로 0이 되어 내부 무제한 profile이 되는 설정을 거부한다."""
    from fss_arm.commands import position_profiles

    v, a = position_profiles(mapping, [0.3] * 3, [1.0] * 3)
    assert min(v) > 0 and min(a) > 0
    with pytest.raises(ValueError):
        position_profiles(mapping, [0.001] * 3, [1.0] * 3)
    with pytest.raises(ValueError):
        position_profiles(mapping, [0.3] * 3, [0.001] * 3)
