"""관절 위치·속도·전류와 XM430 레지스터 간 변환 및 명령 수명 검사."""

from dataclasses import dataclass
from math import floor, isfinite, pi

import numpy as np

VELOCITY_RAD_S_PER_LSB = 0.229 * 2 * pi / 60
POSITION_RAD_PER_TICK = 2 * pi / 4096
CURRENT_A_PER_LSB = 0.00269
ACCELERATION_RAD_S2_PER_LSB = 214.577 * 2 * pi / 3600
COMMAND_MODES = {"velocity": 1, "position": 4, "current": 0}
JOINT_NAMES = ("shoulder", "elbow", "wrist")


def vector3(value: object, name: str) -> np.ndarray:
    """유한한 3축 벡터만 받는다."""
    array = np.asarray(value, dtype=float)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{name}: finite three-vector required")
    return array


@dataclass(frozen=True)
class JointMapping:
    """감속비는 Dynamixel 출력축 회전/관절 회전 비율이다."""

    ids: tuple[int, int, int]
    directions: tuple[int, int, int]
    gear_ratios: tuple[float, float, float]
    zero_ticks: tuple[int, int, int]
    velocity_limits: tuple[float, float, float]
    lower_limits: tuple[float, float, float]
    upper_limits: tuple[float, float, float]

    def __post_init__(self) -> None:
        for name in self.__dataclass_fields__:
            vector3(getattr(self, name), name)
        if len(set(self.ids)) != 3 or any(int(i) != i or not 0 <= i <= 252 for i in self.ids):
            raise ValueError("three unique Dynamixel IDs in [0,252] required")
        if any(d not in (-1, 1) for d in self.directions):
            raise ValueError("joint directions must be -1 or +1")
        if any(g <= 0 for g in self.gear_ratios) or any(v <= 0 for v in self.velocity_limits):
            raise ValueError("gear ratios and velocity limits must be positive")
        if any(int(t) != t for t in self.zero_ticks):
            raise ValueError("zero ticks must be integers")
        if np.any(np.asarray(self.lower_limits) >= self.upper_limits):
            raise ValueError("invalid joint angle limits")

    def encode_velocity(self, velocity: object) -> tuple[int, int, int]:
        """rad/s를 부호 있는 32비트 Goal Velocity 값으로 양자화한다."""
        u = vector3(velocity, "velocity")
        if np.any(np.abs(u) > np.asarray(self.velocity_limits) + 1e-9):
            raise ValueError("joint velocity limit exceeded")
        raw = u * self.directions * self.gear_ratios / VELOCITY_RAD_S_PER_LSB
        # 물리 속도 한계 바깥으로 반올림하지 않도록 표현 가능한 최댓값을 제한한다.
        caps = np.floor(
            np.asarray(self.velocity_limits) * self.gear_ratios / VELOCITY_RAD_S_PER_LSB
        )
        if np.any(caps > 1023):
            raise ValueError("XM430 velocity register range exceeded by configured limits")
        rounded = np.sign(raw) * np.floor(np.abs(raw) + 0.5)
        return tuple(int(x) for x in np.clip(rounded, -caps, caps))

    def decode_velocity(self, raw: object) -> np.ndarray:
        """Present Velocity 또는 양자화된 명령을 rad/s로 환산한다."""
        return (
            vector3(raw, "raw velocity")
            * VELOCITY_RAD_S_PER_LSB
            / self.gear_ratios
            * self.directions
        )

    def decode_position(self, raw: object) -> np.ndarray:
        """다회전 위치를 wrapping 없이 관절 상대각으로 변환한다."""
        return (
            (vector3(raw, "raw position") - self.zero_ticks)
            * POSITION_RAD_PER_TICK
            / self.gear_ratios
            * self.directions
        )

    def encode_position(self, position: object) -> tuple[int, int, int]:
        """관절각을 다회전 Goal Position으로 변환하고 양자화 후에도 한계 안에 둔다."""
        q = vector3(position, "position")
        self.check_position(q)
        scale = np.asarray(self.directions) * self.gear_ratios / POSITION_RAD_PER_TICK
        bounds = np.array([self.lower_limits, self.upper_limits]) * scale + self.zero_ticks
        lower = np.maximum(np.ceil(bounds.min(axis=0)), -1048575)
        upper = np.minimum(np.floor(bounds.max(axis=0)), 1048575)
        raw = q * scale + self.zero_ticks
        if np.any(lower > upper) or np.any(np.abs(raw) > 1048575):
            raise ValueError("extended position register range exceeded")
        return tuple(int(x) for x in np.clip(np.rint(raw), lower, upper))

    def check_position(self, position: object) -> None:
        """측정 관절각이 설정 한계 안에 있는지 확인한다."""
        q = vector3(position, "position")
        if np.any(q < self.lower_limits) or np.any(q > self.upper_limits):
            raise ValueError("joint position limit exceeded")


class CommandGate:
    """ACTIVE heartbeat와 명령 만료를 검사하고 fault를 유지한다."""

    def __init__(self, mode_timeout_s: float, command_timeout_s: float) -> None:
        if not all(isfinite(x) and x > 0 for x in (mode_timeout_s, command_timeout_s)):
            raise ValueError("positive finite timeouts required")
        self.mode_timeout_s = mode_timeout_s
        self.command_timeout_s = command_timeout_s
        self.active = False
        self.mode_received = float("-inf")
        self.command_received = float("-inf")
        self.fault = ""

    def update_mode(self, active: bool, now: float) -> None:
        """모드 메시지 수신은 로컬 monotonic 시간으로 기록한다."""
        self.active, self.mode_received = active, now
        if not active:
            self.command_received = float("-inf")

    def mode_ok(self, now: float) -> bool:
        """새 ACTIVE heartbeat가 있을 때만 실행을 허용한다."""
        return self.active and 0 <= now - self.mode_received <= self.mode_timeout_s

    def ready(self, now: float) -> bool:
        """명령과 모드가 모두 유효해야 출력한다."""
        return (
            not self.fault
            and self.mode_ok(now)
            and 0 <= now - self.command_received <= self.command_timeout_s
        )


def signed32(value: int) -> int:
    """SDK의 unsigned read 결과를 2의 보수 signed 값으로 해석한다."""
    return value - (1 << 32) if value & (1 << 31) else value


def signed16(value: int) -> int:
    """2-byte 전류를 signed로 해석한다."""
    return value - (1 << 16) if value & (1 << 15) else value


def encode_current(mapping: JointMapping, current: object, limits: object) -> tuple[int, int, int]:
    """관절 +방향 기준 모터 전류 A를 변환한다. 외부 감속비는 적용하지 않는다."""
    values, caps_a = vector3(current, "current"), vector3(limits, "current limits")
    if np.any(caps_a <= 0) or np.any(caps_a > 1193 * CURRENT_A_PER_LSB):
        raise ValueError("current limits outside XM430 range")
    if np.any(np.abs(values) > caps_a):
        raise ValueError("current limit exceeded")
    raw = values * mapping.directions / CURRENT_A_PER_LSB
    caps = np.floor(caps_a / CURRENT_A_PER_LSB)
    if np.any(caps < 1):
        raise ValueError("current limit below one register unit")
    rounded = np.sign(raw) * np.floor(np.abs(raw) + 0.5)
    return tuple(int(x) for x in np.clip(rounded, -caps, caps))


def decode_current(mapping: JointMapping, raw: object) -> np.ndarray:
    """signed 전류 레지스터를 관절 +방향 기준 A로 변환한다."""
    return vector3(raw, "raw current") * CURRENT_A_PER_LSB * mapping.directions


def position_profiles(mapping: JointMapping, velocity: object, acceleration: object) -> tuple:
    """위치 모드의 유한한 내부 속도·가속도 profile을 장치 단위로 내림한다."""
    v, a = (
        vector3(velocity, "position profile velocity"),
        vector3(acceleration, "position profile acceleration"),
    )
    if np.any(v <= 0) or np.any(v > mapping.velocity_limits) or np.any(a <= 0):
        raise ValueError("invalid position profile limits")
    raw_v = np.floor(v * mapping.gear_ratios / VELOCITY_RAD_S_PER_LSB)
    raw_a = np.floor(a * mapping.gear_ratios / ACCELERATION_RAD_S2_PER_LSB)
    if np.any(raw_v < 1) or np.any(raw_a < 1) or np.any(raw_v > 1023) or np.any(raw_a > 32767):
        raise ValueError("position profile is outside finite register range")
    return tuple(int(x) for x in raw_v), tuple(int(x) for x in raw_a)


def watchdog_ticks(timeout_s: float) -> int:
    """20 ms 단위 watchdog 값은 요청한 timeout보다 길어지지 않는다."""
    if not isfinite(timeout_s) or not 0.02 <= timeout_s <= 2.54:
        raise ValueError("watchdog timeout must be in [0.02,2.54] s")
    return floor(timeout_s / 0.02 + 1e-12)


def return_delay_ticks(delay_us: int) -> int:
    """응답 지연 설정을 2 us 단위 레지스터 값으로 검증·변환한다."""
    if (
        isinstance(delay_us, bool)
        or not isinstance(delay_us, int)
        or not 0 <= delay_us <= 508
        or delay_us % 2
    ):
        raise ValueError("return delay must be an even integer in [0,508] us")
    return delay_us // 2
