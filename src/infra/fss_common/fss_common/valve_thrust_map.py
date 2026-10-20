"""밸브별 정적 입력(0..1) ↔ 추력[N] 구간 선형 맵."""

from pathlib import Path

import numpy as np
import yaml

from fss_common.paths import config_file


class ValveThrustMap:
    """공통 입력 축과 밸브 순서대로 나열한 단조 추력 곡선."""

    def __init__(self, duty, thrust_n, valve_count: int = 8) -> None:
        self._duty = np.array(duty, dtype=float, copy=True)
        self._thrust = np.array(thrust_n, dtype=float, copy=True)
        if (
            valve_count <= 0
            or self._duty.ndim != 1
            or self._duty.size < 2
            or self._thrust.shape != (valve_count, self._duty.size)
        ):
            raise ValueError("맵은 입력 축(2점 이상)과 밸브별 동일 길이의 추력 행이 필요하다")
        if not np.all(np.isfinite(self._duty)) or not np.all(np.isfinite(self._thrust)):
            raise ValueError("밸브 맵에 NaN 또는 Inf가 포함되어 있다")
        if self._duty[0] != 0.0 or self._duty[-1] != 1.0 or np.any(np.diff(self._duty) <= 0):
            raise ValueError("밸브 입력 축은 0에서 1까지 엄격히 증가해야 한다")
        if (
            np.any(self._thrust[:, 0] != 0.0)
            or np.any(self._thrust[:, -1] <= 0.0)
            or np.any(np.diff(self._thrust, axis=1) < 0.0)
        ):
            raise ValueError("추력 곡선은 0에서 시작해 감소 없이 양의 최대 추력까지 이어져야 한다")

    @classmethod
    def from_yaml(cls, path: str | Path, valve_count: int = 8) -> "ValveThrustMap":
        with open(path, encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
        if not isinstance(data, dict) or not {"duty", "thrust_n"} <= data.keys():
            raise ValueError("밸브 맵 YAML에는 duty와 thrust_n이 필요하다")
        return cls(data["duty"], data["thrust_n"], valve_count)

    @property
    def max_thrust_n(self) -> np.ndarray:
        return self._thrust[:, -1].copy()

    def _values(self, values) -> np.ndarray:
        values = np.asarray(values, dtype=float)
        if values.shape != (len(self._thrust),) or not np.all(np.isfinite(values)):
            raise ValueError("입력은 밸브 개수와 같은 길이의 유한한 값이어야 한다")
        return values

    def thrust_from_duty(self, duty) -> np.ndarray:
        """입력을 0..1로 제한하고 정적 추력[N]을 보간한다."""
        values = self._values(duty)
        return np.array(
            [
                np.interp(value, self._duty, row)
                for value, row in zip(values, self._thrust, strict=True)
            ]
        )

    def duty_from_thrust(self, thrust_n) -> np.ndarray:
        """가용 추력으로 제한한 뒤 역보간한다. 0N은 항상 입력 0이다."""
        targets = np.clip(self._values(thrust_n), 0.0, self.max_thrust_n)
        result = np.zeros(len(targets))
        for i, (target, row) in enumerate(zip(targets, self._thrust, strict=True)):
            if target == 0.0:
                continue
            # 평탄 구간에서는 목표 추력을 내는 가장 작은 입력을 선택한다.
            hi = int(np.searchsorted(row, target, side="left"))
            fraction = (target - row[hi - 1]) / (row[hi] - row[hi - 1])
            result[i] = self._duty[hi - 1] + fraction * (self._duty[hi] - self._duty[hi - 1])
        return result


def load_valve_thrust_map(filename: str, valve_count: int = 8) -> ValveThrustMap | None:
    """빈 설정은 맵 미사용, 그 외에는 config/ 기준 YAML을 검증해 읽는다."""
    if not filename:
        return None
    return ValveThrustMap.from_yaml(
        config_file(*filename.replace("\\", "/").split("/")), valve_count
    )
