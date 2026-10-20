"""선형, 카탈로그, 밸브별 입력→추력 맵 모델."""

import math
from collections import deque

import numpy as np
from fss_common.valve_thrust_map import ValveThrustMap
from numpy.typing import NDArray

from fss_sim.catalog_curve import PVQ30CatalogCurve


class ValveModel:
    """비례밸브의 선형·카탈로그·정적 추력 맵 모델."""

    VALID_MODES = {"ideal", "real", "mapped"}

    def __init__(
        self,
        n_valves: int = 8,
        max_thrust_per_valve: float = 1.0,
        model_mode: str = "ideal",
        catalog_curve: PVQ30CatalogCurve | None = None,
        pressure_diff_mpa: float = 0.55158,
        signal_current_min_ma: float = 4.0,
        signal_current_max_ma: float = 20.0,
        signal_to_coil_current_gain: float = 10.0,
        input_delay_ms: float = 0.0,
        tau_valve_ms: float = 0.0,
        thrust_map: ValveThrustMap | None = None,
    ) -> None:
        """밸브 모델을 초기화한다.

        Args:
            n_valves: 밸브 개수
            max_thrust_per_valve: 카탈로그 최대 압력에서 전개방 시 노즐 추력 [N]
            model_mode: ``ideal``, ``real`` 또는 ``mapped``
            thrust_map: ``mapped`` 모드의 밸브별 정적 입력→추력 맵
            catalog_curve: ``real`` 모드의 PVQ30 φ1.6 유량 곡선
            pressure_diff_mpa: 밸브 입출구 압력차 [MPa]
            signal_current_min_ma: 듀티 0에 대응하는 X33 입력 [mA]
            signal_current_max_ma: 듀티 1에 대응하는 X33 입력 [mA]
            signal_to_coil_current_gain: X33 입력→표준 코일 등가 전류 비
            input_delay_ms: 확인된 명령 전달 순수 지연 [ms]
            tau_valve_ms: 확인된 밸브 기계 응답의 1차 시상수 [ms]

        Raises:
            ValueError: 모드 또는 물리 파라미터가 유효하지 않을 때
        """
        if n_valves <= 0:
            raise ValueError("밸브 개수는 양수여야 한다")
        if model_mode not in self.VALID_MODES:
            raise ValueError(
                f"지원하지 않는 밸브 모델 모드: {model_mode!r} "
                f"(허용: {sorted(self.VALID_MODES)})"
            )
        if not math.isfinite(max_thrust_per_valve) or max_thrust_per_valve <= 0.0:
            raise ValueError("밸브당 최대 추력은 유한한 양수여야 한다")
        for name, value in (
            ("압력차", pressure_diff_mpa),
            ("X33 최소 입력 전류", signal_current_min_ma),
            ("X33 최대 입력 전류", signal_current_max_ma),
            ("X33-코일 전류 이득", signal_to_coil_current_gain),
            ("입력 지연", input_delay_ms),
            ("밸브 시상수", tau_valve_ms),
        ):
            if not math.isfinite(value):
                raise ValueError(f"{name}는 유한해야 한다")
        if pressure_diff_mpa <= 0.0:
            raise ValueError("압력차는 양수여야 한다")
        if signal_current_min_ma < 0.0:
            raise ValueError("X33 최소 입력 전류는 0 이상이어야 한다")
        if signal_current_max_ma <= signal_current_min_ma:
            raise ValueError("X33 최대 입력 전류는 최소 입력 전류보다 커야 한다")
        if signal_to_coil_current_gain <= 0.0:
            raise ValueError("X33-코일 전류 이득은 양수여야 한다")
        if input_delay_ms < 0.0:
            raise ValueError("입력 지연은 0 이상이어야 한다")
        if tau_valve_ms < 0.0:
            raise ValueError("밸브 시상수는 0 이상이어야 한다")
        if model_mode == "real" and catalog_curve is None:
            raise ValueError("real 밸브 모드에는 카탈로그 유량 곡선이 필요하다")
        if model_mode == "mapped":
            if thrust_map is None or len(thrust_map.max_thrust_n) != n_valves:
                raise ValueError("mapped 모드에는 밸브 개수가 일치하는 추력 맵이 필요하다")

        self._n = n_valves
        self._max_thrust = max_thrust_per_valve
        self._mode = model_mode
        self._curve = catalog_curve if model_mode == "real" else None
        self._thrust_map = thrust_map if model_mode == "mapped" else None
        if self._thrust_map is not None:
            self._max_thrust = self._thrust_map.max_thrust_n
        self._pressure_diff_mpa = pressure_diff_mpa
        self._signal_min_ma = signal_current_min_ma
        self._signal_max_ma = signal_current_max_ma
        self._signal_to_coil_gain = signal_to_coil_current_gain
        self._input_delay = input_delay_ms * 1e-3
        self._tau_valve = tau_valve_ms * 1e-3

        self._full_scale_flow_lpm = 0.0
        if self._curve is not None:
            minimum_pressure_mpa, reference_pressure_mpa = self._curve.pressure_range_mpa
            if not minimum_pressure_mpa <= self._pressure_diff_mpa <= reference_pressure_mpa:
                raise ValueError(
                    f"압력차 {self._pressure_diff_mpa}MPa는 카탈로그 범위 "
                    f"[{minimum_pressure_mpa}, {reference_pressure_mpa}]MPa 밖이다"
                )
            maximum_coil_current = self._signal_max_ma * self._signal_to_coil_gain
            self._full_scale_flow_lpm = float(
                self._curve.flow_lpm(
                    np.asarray([maximum_coil_current]),
                    reference_pressure_mpa,
                    np.asarray([True]),
                )[0]
            )
            if self._full_scale_flow_lpm <= 0.0:
                raise ValueError("기준 압력·최대 입력에서 카탈로그 유량이 0이다")

        self._time_sec = 0.0
        self._pending_commands: deque[tuple[float, NDArray[np.float64]]] = deque()
        self._last_received = np.zeros(n_valves, dtype=float)
        self._delayed_command = np.zeros(n_valves, dtype=float)
        self._last_coil_current_ma = np.full(
            n_valves,
            self._signal_min_ma * self._signal_to_coil_gain,
            dtype=float,
        )
        self._increasing = np.ones(n_valves, dtype=bool)
        self._stage = np.zeros(n_valves, dtype=float)

    @property
    def actual_thrust(self) -> NDArray[np.float64]:
        """현재 실제 추력 [N]을 반환한다."""
        return self._stage * self._max_thrust

    @property
    def actual_flow_lpm(self) -> NDArray[np.float64]:
        """``real`` 모드의 현재 카탈로그 등가 유량 [L/min]을 반환한다."""
        if self._curve is None:
            raise RuntimeError("ideal 모드에는 카탈로그 등가 유량이 없다")
        return self._stage * self._full_scale_flow_lpm

    @property
    def full_scale_flow_lpm(self) -> float:
        """카탈로그 최대 압력·최대 X33 입력의 기준 유량 [L/min]을 반환한다."""
        if self._curve is None:
            raise RuntimeError("ideal 모드에는 카탈로그 전개방 유량이 없다")
        return self._full_scale_flow_lpm

    def _delay_command(self, command: NDArray[np.float64], dt: float) -> NDArray[np.float64]:
        """입력 명령을 시뮬레이션 시간 기준으로 순수 지연시킨다."""
        if not np.array_equal(command, self._last_received):
            activation_time = self._time_sec + self._input_delay
            self._pending_commands.append((activation_time, command.copy()))
            self._last_received = command.copy()

        while self._pending_commands and self._pending_commands[0][0] <= self._time_sec + 1e-12:
            _, self._delayed_command = self._pending_commands.popleft()

        self._time_sec += dt
        return self._delayed_command

    def _catalog_target(self, command: NDArray[np.float64]) -> NDArray[np.float64]:
        """X33 신호를 압력별 상승·하강 카탈로그 유량으로 변환한다."""
        if self._curve is None:
            raise RuntimeError("카탈로그 곡선 없이 real 밸브 모델을 계산할 수 없다")

        signal_ma = self._signal_min_ma + command * (self._signal_max_ma - self._signal_min_ma)
        coil_current_ma = signal_ma * self._signal_to_coil_gain
        delta = coil_current_ma - self._last_coil_current_ma
        self._increasing = np.where(
            delta > 1e-12,
            True,
            np.where(delta < -1e-12, False, self._increasing),
        )
        self._last_coil_current_ma = coil_current_ma.copy()
        flow_lpm = self._curve.flow_lpm(
            coil_current_ma,
            self._pressure_diff_mpa,
            self._increasing,
        )
        return flow_lpm / self._full_scale_flow_lpm

    def step(self, duty_commands: list[float], dt: float) -> NDArray[np.float64]:
        """한 시간 스텝만큼 밸브 상태를 갱신한다."""
        command = np.asarray(duty_commands, dtype=float)
        if command.shape != (self._n,):
            raise ValueError(f"밸브 명령은 정확히 {self._n}개여야 한다")
        if not np.all(np.isfinite(command)):
            raise ValueError("밸브 명령에 NaN 또는 Inf가 포함되어 있다")
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError("시간 간격은 유한한 양수여야 한다")
        command = np.clip(command, 0.0, 1.0)

        if self._mode == "ideal":
            self._stage = command.copy()
            self._time_sec += dt
            return self.actual_thrust

        delayed = self._delay_command(command, dt)
        target = (
            self._thrust_map.thrust_from_duty(delayed) / self._max_thrust
            if self._thrust_map is not None else self._catalog_target(delayed)
        )

        if self._tau_valve > 0.0:
            alpha = -math.expm1(-dt / self._tau_valve)
            self._stage += alpha * (target - self._stage)
        else:
            self._stage = target.copy()
        return self.actual_thrust

    def reset(self) -> None:
        """밸브 상태와 지연·곡선 방향 이력을 초기화한다."""
        self._time_sec = 0.0
        self._pending_commands.clear()
        self._last_received[:] = 0.0
        self._delayed_command[:] = 0.0
        self._last_coil_current_ma[:] = self._signal_min_ma * self._signal_to_coil_gain
        self._increasing[:] = True
        self._stage[:] = 0.0
