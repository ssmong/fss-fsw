"""XM430-W350 Protocol 2.0 통신 경계. 직렬 포트는 명시적 연결 때만 연다."""

from inspect import signature
from types import ModuleType

import numpy as np

from fss_arm.commands import (
    COMMAND_MODES,
    JointMapping,
    decode_current,
    position_profiles,
    return_delay_ticks,
    signed16,
    signed32,
    watchdog_ticks,
)

# 오류, watchdog, torque, 측정 상태와 세 모드의 직전 목표를 한 상태 블록에 배치한다.
STATE_ADDRESSES = (
    70,
    98,
    64,
    *range(128, 136),
    *range(104, 108),
    126,
    127,
    102,
    103,
    *range(116, 120),
)
GOAL_REGISTERS = {"velocity": (104, 4), "position": (116, 4), "current": (102, 2)}
INDIRECT_ADDRESS = 168
INDIRECT_DATA = 224


class DynamixelBus:
    """위치·속도·전류 목표 SyncWrite와 측정 상태 SyncRead를 제공한다."""

    def __init__(
        self,
        mapping: JointMapping,
        port: str,
        baudrate: int,
        model_numbers: list[int],
        watchdog_s: float,
        sdk: ModuleType | None = None,
        return_delay_us: int = 0,
        command_mode: str = "velocity",
        profile_velocity: tuple = (0.3, 0.3, 0.3),
        profile_acceleration: tuple = (1.0, 1.0, 1.0),
    ) -> None:
        if model_numbers != [1020, 1020, 1020]:
            raise ValueError("this control table supports three XM430-W350 (model 1020) only")
        if not port or baudrate <= 0:
            raise ValueError("explicit port and positive baudrate required")
        delay_ticks = return_delay_ticks(return_delay_us)
        if command_mode not in COMMAND_MODES:
            raise ValueError("unknown arm command mode")
        self.profile_velocity, self.profile_acceleration = position_profiles(
            mapping, profile_velocity, profile_acceleration
        )
        if sdk is None:
            import dynamixel_sdk as sdk
        self.sdk = sdk
        self.mapping = mapping
        self.port = sdk.PortHandler(port)
        self.packet = sdk.PacketHandler(2.0)
        self.rx_options = (
            {"fast_option": False}
            if "fast_option" in signature(self.packet.rxPacket).parameters
            else {}
        )
        self.baudrate = baudrate
        self.watchdog = watchdog_ticks(watchdog_s)
        self.connected = False
        self.enabled = False
        self.velocity_caps: list[int] = []
        self.return_delay_ticks = delay_ticks
        self.reader = None
        self.last_goal: tuple[int, int, int] | None = None
        self.command_mode = command_mode
        self.mode_ready = False
        self.current_caps: list[int] = []
        self.current = np.zeros(3)

    def _check(self, result: int, error: int, operation: str) -> None:
        if result != self.sdk.COMM_SUCCESS or error:
            raise RuntimeError(f"Dynamixel {operation}: communication={result}, device={error}")

    def _read(self, motor_id: int, address: int, size: int) -> int:
        value, result, error = getattr(self.packet, f"read{size}ByteTxRx")(
            self.port, motor_id, address
        )
        self._check(result, error, f"read ID={motor_id} address={address}")
        return value

    def _write(self, motor_id: int, address: int, size: int, value: int) -> None:
        result, error = getattr(self.packet, f"write{size}ByteTxRx")(
            self.port, motor_id, address, value
        )
        self._check(result, error, f"write ID={motor_id} address={address}")

    def connect(self) -> None:
        """전 모터 사전 검사 후 토크 OFF에서 통신 지연과 indirect 상태 블록을 설정한다."""
        if self.connected:
            raise RuntimeError("Dynamixel bus already connected")
        if not self.port.openPort():
            raise RuntimeError("Dynamixel port open failed")
        try:
            self.velocity_caps.clear()
            self.current_caps.clear()
            if not self.port.setBaudRate(self.baudrate):
                raise RuntimeError("Dynamixel baudrate setup failed")
            for motor_id in self.mapping.ids:
                model, result, error = self.packet.ping(self.port, motor_id)
                self._check(result, error, f"ping ID={motor_id}")
                if model != 1020:
                    raise ValueError(f"unexpected motor model {model} on ID {motor_id}")
                # 시간 기반 profile과 장치 reverse를 끄고 소프트웨어 보정 부호를 사용한다.
                for address, size, expected in (
                    (64, 1, 0),
                    (10, 1, 0),
                    (68, 1, 2),
                ):
                    if self._read(motor_id, address, size) != expected:
                        raise ValueError(
                            f"ID {motor_id}: configure register {address}={expected} first"
                        )
                if self._read(motor_id, 70, 1) != 0 or self._read(motor_id, 98, 1) != 0:
                    raise ValueError(f"ID {motor_id}: hardware/watchdog fault or active watchdog")
                self.velocity_caps.append(self._read(motor_id, 44, 4))
                self.current_caps.append(self._read(motor_id, 38, 2))
            # 전 축의 설정을 확인한 뒤 변경한다. EEPROM은 차이가 있을 때만 쓴다.
            for motor_id in self.mapping.ids:
                self._configure(motor_id, 9, 1, self.return_delay_ticks)
                for index, address in enumerate(STATE_ADDRESSES):
                    self._configure(motor_id, INDIRECT_ADDRESS + 2 * index, 2, address)
            self.reader = self.sdk.GroupSyncRead(
                self.port, self.packet, INDIRECT_DATA, len(STATE_ADDRESSES)
            )
            for motor_id in self.mapping.ids:
                if not self.reader.addParam(motor_id):
                    raise RuntimeError(f"SyncRead addParam failed on ID {motor_id}")
            self.last_goal = None
            self._configure_mode(self.command_mode)
            self.connected = True
        except Exception:
            self.port.closePort()
            raise

    def set_mode(self, mode: str) -> None:
        """토크 OFF와 정지 상태에서만 전 축의 명령 모드를 전환한다."""
        if not self.connected:
            raise RuntimeError("Dynamixel bus not connected")
        self._configure_mode(mode)

    def _configure_mode(self, mode: str) -> None:
        """게인을 보존하며 모드·profile을 설정하고 위치 카운터 연속성을 확인한다."""
        if mode not in COMMAND_MODES or self.enabled:
            raise ValueError("valid mode and disabled torque required")
        before = []
        for motor_id in self.mapping.ids:
            if (
                self._read(motor_id, 64, 1)
                or self._read(motor_id, 98, 1)
                or self._read(motor_id, 70, 1)
            ):
                raise ValueError("torque off and clear hardware/watchdog required for mode change")
            before.append(signed32(self._read(motor_id, 132, 4)))
        self.mapping.check_position(self.mapping.decode_position(before))
        velocity = [signed32(self._read(i, 128, 4)) for i in self.mapping.ids]
        if np.max(np.abs(self.mapping.decode_velocity(velocity))) > 0.02:
            raise ValueError("arm must be stopped for mode change")
        self.mode_ready = False
        for index, motor_id in enumerate(self.mapping.ids):
            if self._read(motor_id, 11, 1) != COMMAND_MODES[mode]:
                gains = {a: self._read(motor_id, a, 2) for a in (76, 78, 80, 82, 84, 88, 90, 100)}
                self._configure(motor_id, 11, 1, COMMAND_MODES[mode])
                for address, value in gains.items():
                    self._configure(motor_id, address, 2, value)
            v = self.profile_velocity[index] if mode == "position" else 0
            a = self.profile_acceleration[index] if mode == "position" else 0
            if v > self.velocity_caps[index]:
                raise ValueError("position profile exceeds hardware velocity limit")
            self._configure(motor_id, 108, 4, a)
            self._configure(motor_id, 112, 4, v)
            after = signed32(self._read(motor_id, 132, 4))
            if abs(after - before[index]) > 2:
                raise RuntimeError(
                    "position counter changed during mode switch; "
                    "recalibrate zero before restarting"
                )
        self.command_mode = mode
        self.last_goal = None
        self.mode_ready = True

    def _configure(self, motor_id: int, address: int, size: int, value: int) -> None:
        """설정 차이가 있으면 기록하며 실제 반영을 확인한다."""
        if self._read(motor_id, address, size) != value:
            self._write(motor_id, address, size, value)
        if self._read(motor_id, address, size) != value:
            raise RuntimeError(f"configuration readback mismatch ID={motor_id} address={address}")

    def write_velocity(self, raw: tuple[int, int, int]) -> None:
        """3축을 한 패킷으로 전송하며 통신 실패는 호출자에게 전파한다."""
        if self.command_mode != "velocity":
            raise ValueError("velocity command requires velocity mode")
        if len(raw) != 3 or any(
            abs(v) > limit for v, limit in zip(raw, self.velocity_caps, strict=True)
        ):
            raise ValueError("hardware velocity limit exceeded")
        if not self.enabled and any(raw):
            raise RuntimeError("Dynamixel torque not enabled")
        self._send_goal(raw)

    def write_position(self, raw: tuple[int, int, int]) -> None:
        """다회전 위치 목표를 전 축에 전송한다."""
        if self.command_mode != "position" or not self.enabled:
            raise ValueError("enabled position mode required")
        self.mapping.check_position(self.mapping.decode_position(raw))
        if any(abs(v) > 1048575 for v in raw):
            raise ValueError("extended position range exceeded")
        self._send_goal(raw)

    def write_current(self, raw: tuple[int, int, int]) -> None:
        """signed 전류 목표를 장치 Current Limit 이내에서 전송한다."""
        if self.command_mode != "current" or (not self.enabled and any(raw)):
            raise ValueError("enabled current mode required")
        if len(raw) != 3 or any(
            abs(v) > cap for v, cap in zip(raw, self.current_caps, strict=True)
        ):
            raise ValueError("hardware current limit exceeded")
        self._send_goal(raw)

    def _send_goal(self, raw: tuple[int, int, int]) -> None:
        """선택 모드의 레지스터에 SyncWrite하며 다음 읽기에서 적용을 확인한다."""
        if not self.connected or not self.mode_ready:
            raise RuntimeError("Dynamixel bus/mode not ready")
        address, size = GOAL_REGISTERS[self.command_mode]
        group = self.sdk.GroupSyncWrite(self.port, self.packet, address, size)
        try:
            for motor_id, value in zip(self.mapping.ids, raw, strict=True):
                data = list(int(value).to_bytes(size, byteorder="little", signed=True))
                if not group.addParam(motor_id, data):
                    raise RuntimeError(f"SyncWrite addParam failed on ID {motor_id}")
            self._check(group.txPacket(), 0, f"SyncWrite {self.command_mode}")
            self.last_goal = tuple(raw)
        finally:
            group.clearParam()

    def enable(self) -> None:
        """정지 명령과 장치 watchdog을 먼저 설정한 뒤 토크를 켠다."""
        if not self.mode_ready:
            raise RuntimeError("mode configuration incomplete; reconnect required")
        neutral = (
            tuple(signed32(self._read(i, 132, 4)) for i in self.mapping.ids)
            if self.command_mode == "position"
            else (0, 0, 0)
        )
        if self.command_mode == "position":
            self.mapping.check_position(self.mapping.decode_position(neutral))
            if any(abs(v) > 1048575 for v in neutral):
                raise ValueError("initial position exceeds extended position range")
        self._send_goal(neutral)
        try:
            address, size = GOAL_REGISTERS[self.command_mode]
            for motor_id, value in zip(self.mapping.ids, neutral, strict=True):
                # SyncWrite는 장치 ACK가 없으므로 enable 전에 개별 쓰기/읽기로 초기 목표를 확인한다.
                self._write(motor_id, address, size, value & ((1 << (size * 8)) - 1))
                if self._read(motor_id, address, size) != value & ((1 << (size * 8)) - 1):
                    raise RuntimeError(f"neutral goal not confirmed on ID {motor_id}")
            for index, motor_id in enumerate(self.mapping.ids):
                self._write(motor_id, 98, 1, self.watchdog)
                self._write(motor_id, 64, 1, 1)
                if self._read(motor_id, 64, 1) != 1:
                    raise RuntimeError(f"torque enable not confirmed on ID {motor_id}")
                if (
                    self.command_mode == "position"
                    and abs(signed32(self._read(motor_id, 132, 4)) - neutral[index]) > 2
                ):
                    raise RuntimeError("position counter changed during torque enable")
            self.enabled = True
        except Exception:
            self.disable()
            raise

    def read_state(self) -> tuple[np.ndarray, np.ndarray]:
        """SyncRead 요청 하나와 3개 응답으로 상태·직전 명령 적용을 검사한다."""
        if not self.connected or self.reader is None:
            raise RuntimeError("Dynamixel bus not connected")
        self._check(self.reader.txPacket(), 0, "SyncRead request")
        positions, velocities, currents = [], [], []
        for index, motor_id in enumerate(self.mapping.ids):
            # readRx의 고정 길이 슬라이스가 짧은 응답의 CRC를 데이터로 포함하지 않게 한다.
            frame, result = self.packet.rxPacket(self.port, **self.rx_options)
            self._check(result, 0, f"SyncRead response ID={motor_id}")
            # SDK는 stuffing 제거 후 리스트 꼬리를 줄이지 않으므로 갱신된 LENGTH를 검사한다.
            if (
                len(frame) < len(STATE_ADDRESSES) + 11
                or frame[4] != motor_id
                or (frame[5] | frame[6] << 8) != len(STATE_ADDRESSES) + 4
            ):
                raise RuntimeError(f"incomplete SyncRead response on ID {motor_id}")
            self._check(result, frame[8], f"SyncRead response ID={motor_id}")
            data = frame[9 : 9 + len(STATE_ADDRESSES)]
            if data[0] or data[1] == 255:
                raise RuntimeError(f"hardware/watchdog fault on ID {motor_id}")
            if self.enabled and data[1] != self.watchdog:
                raise RuntimeError(f"unexpected watchdog setting on ID {motor_id}")
            if self.enabled and data[2] != 1:
                raise RuntimeError(f"unexpected torque disable on ID {motor_id}")
            velocities.append(signed32(int.from_bytes(bytes(data[3:7]), "little")))
            positions.append(signed32(int.from_bytes(bytes(data[7:11]), "little")))
            currents.append(signed16(int.from_bytes(bytes(data[15:17]), "little")))
            offset, length = {"velocity": (11, 4), "current": (17, 2), "position": (19, 4)}[
                self.command_mode
            ]
            value = int.from_bytes(bytes(data[offset : offset + length]), "little")
            goal = signed16(value) if length == 2 else signed32(value)
            if self.enabled and self.last_goal is not None and goal != self.last_goal[index]:
                raise RuntimeError(f"{self.command_mode} goal readback mismatch on ID {motor_id}")
        self.current = decode_current(self.mapping, currents)
        return self.mapping.decode_position(positions), self.mapping.decode_velocity(velocities)

    def disable(self) -> None:
        """한 장치가 실패해도 나머지 장치의 정지를 모두 시도한다."""
        errors = []
        try:
            if self.mode_ready and self.command_mode in ("velocity", "current"):
                self._send_goal((0, 0, 0))
        except Exception as error:
            errors.append(str(error))
        for motor_id in self.mapping.ids:
            try:
                self._write(motor_id, 64, 1, 0)
                if self._read(motor_id, 64, 1) != 0:
                    raise RuntimeError(f"torque disable not confirmed on ID {motor_id}")
            except Exception as error:
                errors.append(str(error))
                # 토크 해제를 확인하지 못한 장치에는 마지막 정지 수단인 watchdog을 남긴다.
                continue
            try:
                self._write(motor_id, 98, 1, 0)
            except Exception as error:
                errors.append(str(error))
        self.enabled = False
        if errors:
            raise RuntimeError("; ".join(errors))

    def close(self) -> None:
        """종료 시 정지를 시도하고 포트를 닫는다."""
        try:
            if self.connected:
                self.disable()
        finally:
            self.port.closePort()
            self.connected = False
