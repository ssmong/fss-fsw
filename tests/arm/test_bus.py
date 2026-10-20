"""SDK 가짜 전송 경계에서 패킷과 실패 시 정지 순서를 검사한다."""

from types import SimpleNamespace

import numpy as np
import pytest
from fss_arm.commands import JointMapping
from fss_arm.dynamixel_bus import INDIRECT_ADDRESS, INDIRECT_DATA, STATE_ADDRESSES, DynamixelBus


class FakeSdk:
    """물리 포트 없이 송수신 실패를 주입하는 SDK fixture."""

    COMM_SUCCESS = 0

    def __init__(self) -> None:
        self.writes = []
        self.params = {}
        self.error_id = None
        self.tx_error = 0
        self.device_error = 0
        self.closed = False
        self.read_calls = []
        self.sync_requests = 0
        self.sync_responses = 0
        self.missing_id = None
        self.short_id = None
        self.drop_write_id = None
        self.ignore_config = None
        self.goal_address = 104
        self.reset_position_on_mode = False
        self.rx_index = 0
        self.values = {
            (i, a): v
            for i in (1, 2, 3)
            for a, v in (
                (64, 0),
                (11, 1),
                (10, 0),
                (108, 0),
                (70, 0),
                (98, 0),
                (44, 200),
                (68, 2),
                (9, 250),
                (38, 1193),
            )
        }

    def close_port(self) -> None:
        self.closed = True

    def port_handler(self, name: str) -> SimpleNamespace:
        return SimpleNamespace(
            openPort=lambda: True, setBaudRate=lambda baud: True, closePort=self.close_port
        )

    def packet_handler(self, protocol: float) -> SimpleNamespace:
        return SimpleNamespace(
            ping=lambda port, motor_id: (1020, 0, 0),
            read1ByteTxRx=self.read,
            read2ByteTxRx=self.read,
            read4ByteTxRx=self.read,
            write1ByteTxRx=self.write,
            write2ByteTxRx=self.write,
            write4ByteTxRx=self.write,
            readRx=self.read_rx,
            rxPacket=self.rx_packet,
            getProtocolVersion=lambda: 2.0,
            syncReadTx=self.sync_read_tx,
        )

    def read(self, port: object, motor_id: int, address: int) -> tuple[int, int, int]:
        self.read_calls.append((motor_id, address))
        return self.values.get((motor_id, address), 0), 0, self.device_error

    def write(self, port: object, motor_id: int, address: int, value: int) -> tuple[int, int]:
        self.writes.append((motor_id, address, value))
        if motor_id == self.error_id:
            return -1, 0
        if (motor_id, address) != self.ignore_config:
            self.values[motor_id, address] = value
            if address == 11:
                for gain in (76, 78, 80, 82, 84, 88, 90, 100):
                    self.values[motor_id, gain] = 0
            if address == 11 and self.reset_position_on_mode:
                self.values[motor_id, 132] = 4096
        return 0, 0

    def sync_read_tx(self, *args: object) -> int:
        """요청 수를 세고 전송 실패를 주입한다."""
        self.sync_requests += 1
        self.rx_index = 0
        return self.tx_error

    def rx_packet(self, port: object, fast_option: bool = False) -> tuple[list[int], int]:
        """SDK 파서가 반환하는 헤더·상태·CRC를 포함한 프레임 형태를 제공한다."""
        self.rx_index += 1
        motor_id = self.rx_index
        data, result, error = self.read_rx(port, motor_id, len(STATE_ADDRESSES))
        frame = [255, 255, 253, 0, motor_id, len(data) + 4, 0, 85, error, *data, 0, 0]
        return frame, result

    def group_sync_read(self, *args: object) -> SimpleNamespace:
        """동일한 indirect 블록에 대한 한 요청을 구성한다."""
        assert args[2:] == (INDIRECT_DATA, len(STATE_ADDRESSES))
        return SimpleNamespace(addParam=lambda motor_id: True, txPacket=self.sync_read_tx)

    def read_rx(self, port: object, motor_id: int, size: int) -> tuple[list[int], int, int]:
        """설정한 indirect 주소를 실제 레지스터 바이트로 해석한다."""
        self.sync_responses += 1
        if motor_id == self.missing_id:
            return [], -3001, 0
        data = []
        for index in range(size):
            address = self.values[motor_id, INDIRECT_ADDRESS + index * 2]
            base = next(
                (
                    b
                    for b, size in ((102, 2), (104, 4), (116, 4), (126, 2), (128, 4), (132, 4))
                    if b <= address < b + size
                ),
                address,
            )
            value = self.values.get((motor_id, base), 0)
            data.append((value >> ((address - base) * 8)) & 255)
        if motor_id == self.short_id:
            data.pop()
        return data, 0, self.device_error

    def group_sync_write(
        self, port: object, packet: object, address: int, size: int
    ) -> SimpleNamespace:
        assert (address, size) in ((104, 4), (116, 4), (102, 2))
        self.goal_address = address
        return SimpleNamespace(
            addParam=self.add_param, txPacket=self.sync_write_tx, clearParam=lambda: None
        )

    def sync_write_tx(self) -> int:
        """쓰기 누락을 선택적으로 주입한다."""
        if self.tx_error == 0:
            for motor_id, data in self.params.items():
                if motor_id != self.drop_write_id:
                    self.values[motor_id, self.goal_address] = int.from_bytes(bytes(data), "little")
        return self.tx_error

    def add_param(self, motor_id: int, data: list[int]) -> bool:
        self.params[motor_id] = data
        return True

    def module(self) -> SimpleNamespace:
        return SimpleNamespace(
            COMM_SUCCESS=0,
            PortHandler=self.port_handler,
            PacketHandler=self.packet_handler,
            GroupSyncWrite=self.group_sync_write,
            GroupSyncRead=self.group_sync_read,
        )


def connected_bus() -> tuple[DynamixelBus, FakeSdk]:
    """설정이 일치하는 3축 가짜 버스를 생성한다."""
    mapping = JointMapping(
        (1, 2, 3), (1, 1, 1), (1, 1, 1), (0, 0, 0), (1, 1, 1), (-2, -2, -2), (2, 2, 2)
    )
    sdk = FakeSdk()
    bus = DynamixelBus(mapping, "FAKE", 1000000, [1020] * 3, 0.1, sdk=sdk.module())
    bus.connect()
    return bus, sdk


def test_no_torque_at_connect_and_zero_before_enable() -> None:
    """통신 설정만 적용하며 명시적 enable이 watchdog 후에 토크를 켠다."""
    bus, sdk = connected_bus()
    assert all(
        a == 9 or INDIRECT_ADDRESS <= a < INDIRECT_ADDRESS + 2 * len(STATE_ADDRESSES)
        for _, a, _ in sdk.writes
    )
    assert all(sdk.values[i, 64] == 0 for i in (1, 2, 3))
    sdk.writes.clear()
    bus.enable()
    assert sdk.params == {1: [0] * 4, 2: [0] * 4, 3: [0] * 4}
    assert sdk.writes[:3] == [(1, 104, 0), (2, 104, 0), (3, 104, 0)]
    assert sdk.writes[3:5] == [(1, 98, 5), (1, 64, 1)]
    bus.write_velocity((-1, 2, -3))
    assert sdk.params[1] == [255] * 4
    assert sdk.params[3] == [253, 255, 255, 255]
    bus.close()
    assert sdk.closed


def test_transmit_and_device_errors_are_not_ignored() -> None:
    """txPacket 오류와 장치 error bit를 모두 전파한다."""
    bus, sdk = connected_bus()
    bus.enable()
    sdk.tx_error = -1001
    with pytest.raises(RuntimeError, match="SyncWrite"):
        bus.write_velocity((1, 1, 1))
    sdk.tx_error = 0
    sdk.device_error = 4
    with pytest.raises(RuntimeError, match="device=4"):
        bus.read_state()


def test_disable_continues_after_one_motor_fails() -> None:
    """첫 장치 정지 실패가 다른 장치의 정지를 막지 않는다."""
    bus, sdk = connected_bus()
    bus.enable()
    sdk.error_id = 1
    with pytest.raises(RuntimeError):
        bus.close()
    assert (2, 64, 0) in sdk.writes and (3, 64, 0) in sdk.writes
    assert (1, 98, 0) not in sdk.writes
    assert sdk.values[1, 98] == 5
    assert not bus.enabled and sdk.closed


def test_watchdog_and_torque_loss_trip() -> None:
    """장치 watchdog 또는 shutdown을 정상 측정값으로 오인하지 않는다."""
    bus, sdk = connected_bus()
    bus.enable()
    sdk.values[2, 98] = 255
    with pytest.raises(RuntimeError, match="watchdog"):
        bus.read_state()
    sdk.values[2, 98] = 5
    sdk.values[2, 64] = 0
    with pytest.raises(RuntimeError, match="torque"):
        bus.read_state()


def test_sync_read_signed_state_and_one_request() -> None:
    """한 요청으로 3축 signed 상태를 반환하고 개별 read를 추가하지 않는다."""
    bus, sdk = connected_bus()
    bus.enable()
    sdk.read_calls.clear()
    sdk.values[1, 128] = (-20) & 0xFFFFFFFF
    sdk.values[2, 132] = (-4096) & 0xFFFFFFFF
    position, velocity = bus.read_state()
    assert sdk.sync_requests == 1 and sdk.sync_responses == 3
    assert sdk.read_calls == []
    np.testing.assert_allclose(position, [0, -2 * np.pi, 0])
    np.testing.assert_allclose(velocity, [-20 * 0.229 * 2 * np.pi / 60, 0, 0])


@pytest.mark.parametrize("failure", ["missing", "short", "transmit", "watchdog_disabled", "goal"])
def test_sync_read_rejects_invalid_cycle(failure: str) -> None:
    """부분 응답·통신 실패·설정 소실·목표 속도 쓰기 누락은 성공으로 반환하지 않는다."""
    bus, sdk = connected_bus()
    bus.enable()
    bus.read_state()
    if failure == "missing":
        sdk.missing_id = 2
    elif failure == "short":
        sdk.short_id = 3
    elif failure == "transmit":
        sdk.tx_error = -1001
    elif failure == "watchdog_disabled":
        sdk.values[2, 98] = 0
    else:
        sdk.drop_write_id = 2
        bus.write_velocity((10, -10, 20))
    with pytest.raises(RuntimeError):
        bus.read_state()


@pytest.mark.parametrize("failure", ["return_level", "mapping", "delay"])
def test_connect_configuration_failure_keeps_torque_off(failure: str) -> None:
    """ACK 설정 및 통신 설정 readback 실패 시 토크를 켜지 않고 포트를 닫는다."""
    bus, sdk = connected_bus()
    bus.close()
    sdk.writes.clear()
    if failure == "return_level":
        sdk.values[3, 68] = 1
    else:
        address = INDIRECT_ADDRESS if failure == "mapping" else 9
        sdk.values[2, address] = 1
        sdk.ignore_config = (2, address)
    with pytest.raises((ValueError, RuntimeError)):
        bus.connect()
    assert sdk.closed and not bus.connected
    assert not any(a == 64 and v == 1 for _, a, v in sdk.writes)
    if failure == "return_level":
        assert sdk.writes == []


def test_official_sdk_sync_read_builder() -> None:
    """설치된 공식 SDK 요청 생성기와 오류를 보존하는 수신 경로를 함께 검사한다."""
    official = pytest.importorskip("dynamixel_sdk")
    bus, sdk = connected_bus()
    bus.close()
    bus.sdk.GroupSyncRead = official.GroupSyncRead
    bus.connect()
    bus.enable()
    bus.write_velocity((-5, 8, 9))
    bus.read_state()
    assert sdk.sync_requests == 1 and sdk.sync_responses == 3
    sdk.device_error = 4
    with pytest.raises(RuntimeError, match="device=4"):
        bus.read_state()


@pytest.mark.parametrize("mode,address,size", [("position", 116, 4), ("current", 102, 2)])
def test_position_and_current_goals(mode: str, address: int, size: int) -> None:
    """각 모드의 목표 주소·signed 값·응답 확인 및 모드별 정지를 검사한다."""
    bus, sdk = connected_bus()
    bus.set_mode(mode)
    assert all(sdk.values[i, 11] == (4 if mode == "position" else 0) for i in (1, 2, 3))
    bus.enable()
    getattr(bus, f"write_{mode}")((-10, 20, -30))
    assert sdk.goal_address == address
    assert sdk.params[1] == list((-10).to_bytes(size, "little", signed=True))
    sdk.values[2, 126] = (-10) & 65535
    bus.read_state()
    assert bus.current[1] == pytest.approx(-0.0269)
    bus.disable()
    if mode == "position":
        # 위치 모드 정지에서 Goal Position=0을 전송해 원점으로 움직이지 않는다.
        assert sdk.values[2, 116] == 20
    else:
        assert sdk.values[2, 102] == 0
    assert all(sdk.values[i, 64] == 0 for i in (1, 2, 3))


def test_position_enable_uses_measured_pose() -> None:
    """기존 목표가 있어도 측정 위치를 모든 모터에 확인한 뒤 토크를 켠다."""
    bus, sdk = connected_bus()
    sdk.values[1, 132] = (-100) & 0xFFFFFFFF
    sdk.values[2, 132] = 200
    bus.set_mode("position")
    sdk.values[1, 116] = 800
    sdk.writes.clear()
    bus.enable()
    assert sdk.values[1, 116] == (-100) & 0xFFFFFFFF
    assert sdk.values[2, 116] == 200
    first_on = next(i for i, (_, a, v) in enumerate(sdk.writes) if a == 64 and v == 1)
    assert {i for i, a, _ in sdk.writes[:first_on] if a == 116} == {1, 2, 3}


def test_mode_change_does_not_run_with_torque_or_position_jump() -> None:
    """토크 ON 전환과 좌표 카운터 초기화 뒤 enable을 거부한다."""
    bus, sdk = connected_bus()
    bus.enable()
    with pytest.raises(ValueError):
        bus.set_mode("current")
    bus.disable()
    sdk.reset_position_on_mode = True
    with pytest.raises(RuntimeError, match="counter changed"):
        bus.set_mode("position")
    with pytest.raises(RuntimeError, match="configuration incomplete"):
        bus.enable()
    assert all(sdk.values[i, 64] == 0 for i in (1, 2, 3))


def test_current_hardware_limit_and_wrong_mode_rejected() -> None:
    """소프트웨어 외에 장치 Current Limit도 검사하고 다른 모드 전송을 거부한다."""
    bus, _ = connected_bus()
    bus.set_mode("current")
    bus.enable()
    bus.current_caps = [5] * 3
    with pytest.raises(ValueError, match="hardware current"):
        bus.write_current((6, 0, 0))
    with pytest.raises(ValueError, match="velocity mode"):
        bus.write_velocity((0, 0, 0))


@pytest.mark.parametrize("mode,address", [("position", 116), ("current", 102)])
def test_new_modes_detect_dropped_goal(mode: str, address: int) -> None:
    """위치·전류 명령이 모터에 적용되지 않은 경우도 검출한다."""
    bus, sdk = connected_bus()
    bus.set_mode(mode)
    bus.enable()
    sdk.drop_write_id = 2
    getattr(bus, f"write_{mode}")((0, 10, 0))
    assert sdk.values[2, address] == 0
    with pytest.raises(RuntimeError, match="readback mismatch"):
        bus.read_state()


def test_mode_switch_restores_gains_and_pwm_limit() -> None:
    """모드 쓰기로 초기화되는 기존 게인과 PWM 제한을 복원한다."""
    bus, sdk = connected_bus()
    original = {a: a + 10 for a in (76, 78, 80, 82, 84, 88, 90, 100)}
    for address, value in original.items():
        sdk.values[1, address] = value
    bus.set_mode("position")
    assert {a: sdk.values[1, a] for a in original} == original
