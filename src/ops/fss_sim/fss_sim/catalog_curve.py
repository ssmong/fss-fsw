"""PVQ30 계열 카탈로그 유량 곡선 로더와 보간기."""

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml  # type: ignore[import-untyped]
from numpy.typing import NDArray
from scipy.interpolate import PchipInterpolator


@dataclass(frozen=True)
class _Branch:
    """한 압력·한 전류 방향의 단조 유량 곡선."""

    current_ma: NDArray[np.float64]
    flow_lpm: NDArray[np.float64]
    interpolator: PchipInterpolator


@dataclass(frozen=True)
class _PressureCurve:
    """동일 압력차에서 측정한 상승·하강 major loop."""

    pressure_diff_mpa: float
    increasing: _Branch
    decreasing: _Branch


class PVQ30CatalogCurve:
    """카탈로그 LUT를 전류·압력 방향으로 보간한다.

    전류축은 디지타이즈 점을 정확히 통과하고 overshoot하지 않는 단조 PCHIP으로
    연결한다. 압력축은 인접한 두 카탈로그 곡선의 유량을 선형 보간하며,
    카탈로그 범위 밖 압력은 조용히 clamp하지 않는다.
    """

    def __init__(self, curves: list[_PressureCurve]) -> None:
        if len(curves) < 2:
            raise ValueError("카탈로그 압력 곡선은 2개 이상이어야 한다")
        ordered = sorted(curves, key=lambda curve: curve.pressure_diff_mpa)
        pressures = np.asarray([curve.pressure_diff_mpa for curve in ordered], dtype=float)
        if not np.all(np.isfinite(pressures)) or np.any(pressures <= 0.0):
            raise ValueError("압력차는 유한한 양수여야 한다")
        if np.any(np.diff(pressures) <= 0.0):
            raise ValueError("압력차 곡선은 중복 없이 증가해야 한다")

        self._curves = tuple(ordered)
        self._pressures = pressures

    @staticmethod
    def _parse_branch(raw: Any, label: str) -> _Branch:
        if not isinstance(raw, dict):
            raise ValueError(f"{label} 분기 데이터가 매핑이 아니다")
        try:
            current = np.asarray(raw["coil_current_ma"], dtype=float)
            flow = np.asarray(raw["flow_lpm"], dtype=float)
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"{label} 분기 배열을 읽을 수 없다") from exc

        if current.ndim != 1 or flow.ndim != 1 or current.size != flow.size:
            raise ValueError(f"{label} 전류·유량 배열 길이가 일치해야 한다")
        if current.size < 2:
            raise ValueError(f"{label} 분기는 2개 이상의 점이 필요하다")
        if not np.all(np.isfinite(current)) or not np.all(np.isfinite(flow)):
            raise ValueError(f"{label} 분기에 NaN 또는 Inf가 있다")
        if np.any(np.diff(current) <= 0.0):
            raise ValueError(f"{label} 전류는 엄격히 증가해야 한다")
        if np.any(np.diff(flow) < 0.0) or np.any(flow < 0.0):
            raise ValueError(f"{label} 유량은 0 이상의 비감소 값이어야 한다")
        if flow[0] != 0.0:
            raise ValueError(f"{label} 첫 유량은 0이어야 한다")
        return _Branch(
            current_ma=current,
            flow_lpm=flow,
            interpolator=PchipInterpolator(current, flow, extrapolate=False),
        )

    @classmethod
    def from_yaml(cls, path: str | Path) -> "PVQ30CatalogCurve":
        """카탈로그 곡선 YAML을 읽고 계약을 검증한다."""
        with open(path, encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
        if not isinstance(data, dict) or not isinstance(data.get("curves"), list):
            raise ValueError("카탈로그 YAML에 curves 목록이 필요하다")

        curves: list[_PressureCurve] = []
        for index, raw in enumerate(data["curves"]):
            if not isinstance(raw, dict):
                raise ValueError(f"curves[{index}]가 매핑이 아니다")
            try:
                pressure = float(raw["pressure_diff_mpa"])
                increasing = cls._parse_branch(raw["increasing"], f"curves[{index}].increasing")
                decreasing = cls._parse_branch(raw["decreasing"], f"curves[{index}].decreasing")
            except KeyError as exc:
                raise ValueError(f"curves[{index}] 필수 키 누락: {exc.args[0]}") from exc
            curves.append(
                _PressureCurve(
                    pressure_diff_mpa=pressure,
                    increasing=increasing,
                    decreasing=decreasing,
                )
            )
        return cls(curves)

    @property
    def pressure_range_mpa(self) -> tuple[float, float]:
        """보간 가능한 카탈로그 압력차 범위를 반환한다."""
        return float(self._pressures[0]), float(self._pressures[-1])

    @staticmethod
    def _branch_flow(branch: _Branch, current_ma: NDArray[np.float64]) -> NDArray[np.float64]:
        clipped = np.clip(current_ma, branch.current_ma[0], branch.current_ma[-1])
        return np.asarray(branch.interpolator(clipped), dtype=float)

    def flow_lpm(
        self,
        coil_current_ma: NDArray[np.float64],
        pressure_diff_mpa: float,
        increasing: NDArray[np.bool_],
    ) -> NDArray[np.float64]:
        """등가 코일 전류에서 방향별 유량을 계산한다.

        Args:
            coil_current_ma: 표준 DC24V 코일 등가 전류 [mA]
            pressure_diff_mpa: 밸브 입출구 압력차 [MPa]
            increasing: 밸브별 현재 증가 방향 여부

        Raises:
            ValueError: 배열·압력·전류가 카탈로그 계약을 벗어날 때
        """
        current = np.asarray(coil_current_ma, dtype=float)
        direction = np.asarray(increasing, dtype=bool)
        if current.shape != direction.shape:
            raise ValueError("코일 전류와 방향 배열 형상이 일치해야 한다")
        if not np.all(np.isfinite(current)) or np.any(current < 0.0):
            raise ValueError("코일 전류는 유한한 0 이상의 값이어야 한다")
        if not np.isfinite(pressure_diff_mpa):
            raise ValueError("압력차는 유한해야 한다")

        p_min, p_max = self.pressure_range_mpa
        if not p_min <= pressure_diff_mpa <= p_max:
            raise ValueError(
                f"압력차 {pressure_diff_mpa}MPa는 카탈로그 범위 " f"[{p_min}, {p_max}]MPa 밖이다"
            )

        upper = int(np.searchsorted(self._pressures, pressure_diff_mpa, side="left"))
        if upper == 0:
            lower = upper = 0
        elif upper == len(self._curves):
            lower = upper = len(self._curves) - 1
        elif self._pressures[upper] == pressure_diff_mpa:
            lower = upper
        else:
            lower = upper - 1

        output = np.empty_like(current)
        for is_increasing in (False, True):
            mask = direction == is_increasing
            if not np.any(mask):
                continue
            branch_name = "increasing" if is_increasing else "decreasing"
            low_branch = getattr(self._curves[lower], branch_name)
            low_flow = self._branch_flow(low_branch, current[mask])
            if lower == upper:
                output[mask] = low_flow
                continue

            high_branch = getattr(self._curves[upper], branch_name)
            high_flow = self._branch_flow(high_branch, current[mask])
            p_low = self._pressures[lower]
            p_high = self._pressures[upper]
            weight = (pressure_diff_mpa - p_low) / (p_high - p_low)
            output[mask] = low_flow + weight * (high_flow - low_flow)
        return output
