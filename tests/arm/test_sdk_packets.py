"""공식 SDK의 실제 패킷 직렬화·수신 파서와 모의 바이트 포트를 연결한다."""

# SDK의 PortHandler 메서드 이름을 그대로 구현한다.
# ruff: noqa: N802

from types import SimpleNamespace

import numpy as np
import pytest
from fss_arm.commands import JointMapping
from fss_arm.dynamixel_bus import INDIRECT_DATA, STATE_ADDRESSES, DynamixelBus

sdk = pytest.importorskip("dynamixel_sdk")


class BytePort:
    """실제 직렬 장치를 열지 않고 SDK에 분할 응답 바이트를 공급한다."""

    def __init__(self) -> None:
        self.is_using = False
        self.frames = []
        self.buffer = []
        self.requests = []

    def clearPort(self) -> None:
        self.buffer = []

    def writePort(self, data: list[int]) -> int:
        self.requests.append(list(data))
        self.buffer = [byte for frame in self.frames for byte in frame]
        return len(data)

    def readPort(self, length: int) -> list[int]:
        count = min(length, 5)
        chunk, self.buffer = self.buffer[:count], self.buffer[count:]
        return chunk

    def setPacketTimeout(self, length: int) -> None:
        pass

    def isPacketTimeout(self) -> bool:
        return not self.buffer


def status_frame(motor_id: int, data: list[int], error: int = 0) -> list[int]:
    """Protocol 2.0 상태 패킷에 byte stuffing과 CRC를 적용한다."""
    packet = sdk.PacketHandler(2.0)
    frame = [255, 255, 253, 0, motor_id, len(data) + 4, 0, 0x55, error, *data, 0, 0]
    frame = packet.addStuffing(frame)
    crc = packet.updateCRC(0, frame, len(frame) - 2)
    frame[-2:] = [crc & 255, crc >> 8]
    return frame


@pytest.mark.parametrize("failure", [None, "crc", "missing", "error", "short"])
def test_official_sdk_packets(failure: str | None) -> None:
    """한 SyncRead 요청과 응답 CRC·길이·장치 오류·누락 처리를 실제 SDK로 검사한다."""
    port = BytePort()
    mapping = JointMapping(
        (1, 2, 3), (1, 1, 1), (1, 1, 1), (0, 0, 0), (1, 1, 1), (-2, -2, -2), (2, 2, 2)
    )
    module = SimpleNamespace(PortHandler=lambda name: port, PacketHandler=sdk.PacketHandler)
    bus = DynamixelBus(mapping, "BYTE_FAKE", 1000000, [1020] * 3, 0.1, sdk=module)
    bus.sdk.COMM_SUCCESS = sdk.COMM_SUCCESS
    bus.connected = bus.enabled = True
    bus.last_goal = (-1, -1, -1)
    bus.reader = sdk.GroupSyncRead(port, bus.packet, INDIRECT_DATA, len(STATE_ADDRESSES))
    for motor_id in (1, 2, 3):
        assert bus.reader.addParam(motor_id)
        # FF FF FD가 포함된 위치로 SDK의 stuffing 제거 경로도 통과시킨다.
        data = [
            0,
            5,
            1,
            *(-2).to_bytes(4, "little", signed=True),
            255,
            255,
            253,
            0,
            255,
            255,
            255,
            255,
        ]
        data.extend([0] * 8)
        if failure == "missing" and motor_id == 3:
            continue
        if failure == "short" and motor_id == 3:
            data.pop()
        frame = status_frame(motor_id, data, 4 if failure == "error" and motor_id == 2 else 0)
        if failure == "crc" and motor_id == 2:
            frame[-1] ^= 1
        port.frames.append(frame)
    if failure:
        with pytest.raises(RuntimeError):
            bus.read_state()
    else:
        position, velocity = bus.read_state()
        np.testing.assert_allclose(position, np.full(3, 0x00FDFFFF * 2 * np.pi / 4096))
        np.testing.assert_allclose(velocity, np.full(3, -2 * 0.229 * 2 * np.pi / 60))
    assert len(port.requests) == 1
    request = port.requests[0]
    assert request[4] == 254 and request[7] == 0x82
    assert request[8:-2] == [224, 0, 23, 0, 1, 2, 3]
