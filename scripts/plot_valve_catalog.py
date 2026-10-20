#!/usr/bin/env python3
"""PVQ30 φ1.6 카탈로그 LUT를 X33 입력 전류 축으로 그린다.

사용법:
    python scripts/plot_valve_catalog.py
    python scripts/plot_valve_catalog.py --output valve_curve.png
    python scripts/plot_valve_catalog.py --pressure 0.55158
"""

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "ops" / "fss_sim"))

from fss_sim.catalog_curve import PVQ30CatalogCurve  # noqa: E402

CURVE_FILE = REPO_ROOT / "config" / "system" / "sim_valve" / "pvq33_5g_16_x33_flow_curve.yaml"
CATALOG_PRESSURES = (0.2, 0.35, 0.5, 0.7)
COLORS = ("#7A7F87", "#2A9D8F", "#E9A23B", "#D95D5D")


def build_figure(pressure_diff_mpa: float) -> tuple[plt.Figure, plt.Axes]:
    """카탈로그 네 압력과 선택한 운용 압력 곡선을 생성한다."""
    curve = PVQ30CatalogCurve.from_yaml(CURVE_FILE)
    signal_ma = np.linspace(4.0, 20.0, 641)
    equivalent_coil_ma = signal_ma * 10.0

    figure, axes = plt.subplots(figsize=(10.5, 6.8))
    for pressure, color in zip(CATALOG_PRESSURES, COLORS, strict=True):
        for increasing, linestyle, direction in (
            (True, "-", "increasing"),
            (False, "--", "decreasing"),
        ):
            flow = curve.flow_lpm(
                equivalent_coil_ma,
                pressure,
                np.full(equivalent_coil_ma.shape, increasing, dtype=bool),
            )
            axes.plot(
                signal_ma,
                flow,
                linestyle,
                color=color,
                linewidth=2.0,
                label=f"{pressure:g} MPa {direction}",
            )

    for increasing, linestyle, direction in (
        (True, "-", "increasing"),
        (False, "--", "decreasing"),
    ):
        flow = curve.flow_lpm(
            equivalent_coil_ma,
            pressure_diff_mpa,
            np.full(equivalent_coil_ma.shape, increasing, dtype=bool),
        )
        axes.plot(
            signal_ma,
            flow,
            linestyle,
            color="#1D3557",
            linewidth=3.2,
            label=f"operating {pressure_diff_mpa:.5f} MPa {direction}",
        )

    axes.set(
        title="PVQ30/PVQ33 φ1.6 catalog major loop — X33 current-axis scaling",
        xlabel="X33 input signal [mA]",
        ylabel="Catalog flow [L/min]",
        xlim=(4.0, 20.0),
        ylim=(0.0, 120.0),
    )
    axes.grid(True, alpha=0.2)
    axes.legend(ncol=2, frameon=False, fontsize=8)
    figure.tight_layout()
    return figure, axes


def main() -> None:
    """CLI 인자를 처리하여 화면 표시 또는 PNG/SVG/PDF 저장을 수행한다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--pressure",
        type=float,
        default=0.55158,
        help="강조할 운용 압력차 [MPa] (기본: 80psi = 0.55158MPa)",
    )
    parser.add_argument("--output", type=Path, help="지정하면 화면 대신 파일로 저장")
    args = parser.parse_args()

    figure, _ = build_figure(args.pressure)
    if args.output is None:
        plt.show()
    else:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(args.output, dpi=180)
        print(args.output.resolve())


if __name__ == "__main__":
    main()
