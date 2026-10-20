#!/usr/bin/env python3
"""raw Vicon 6-DOF로 에어 OFF/ON 자세 변화와 드리프트를 짧게 비교한다.

에어밸브나 추력기를 자동 제어하지 않는다. 부상 측정 중에는 사람이 기체를 회수할
준비를 하고, 안전 이동 반경을 넘으면 즉시 에어를 차단해야 한다.
"""

import argparse
import math
import sys
import time
from dataclasses import replace
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# colcon 소싱 없이도 저장소 소스 트리에서 순수 계산 모듈을 찾는다.
REPO_ROOT = Path(__file__).resolve().parents[2]
SENSOR_PACKAGE = REPO_ROOT / "src" / "hardware" / "fss_sensors"
sys.path.insert(0, str(SENSOR_PACKAGE))

from fss_sensors.float_calibration import (  # noqa: E402
    FloatPoseStats,
    PoseSample,
    calculate_float_pose_stats,
    float_stats_to_dict,
    trim_change_deg,
)
from fss_sensors.imu_transform import normalize_quaternion  # noqa: E402


class ViconPoseCollector(Node):
    """raw Vicon pose를 수집하고 시작점 기준 XY 이동을 감시한다."""

    def __init__(self, topic: str, position_scale: float, max_displacement_m: float) -> None:
        """Vicon 수집 노드를 초기화한다."""
        super().__init__("vicon_float_check")
        self._position_scale = position_scale
        self._max_displacement_m = max_displacement_m
        self.samples: list[PoseSample] = []
        self.first_time: float | None = None
        self.collection_end_time: float | None = None
        self.rejected_count = 0
        self.boundary_exceeded = False
        self._start_xy: tuple[float, float] | None = None
        self._subscription = self.create_subscription(
            PoseStamped,
            topic,
            self._on_pose,
            qos_profile_sensor_data,
        )

    def _on_pose(self, msg: PoseStamped) -> None:
        """유효 pose를 저장하고 안전 이동 반경을 검사한다."""
        position = (
            msg.pose.position.x * self._position_scale,
            msg.pose.position.y * self._position_scale,
            msg.pose.position.z * self._position_scale,
        )
        quaternion_raw = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in (*position, *quaternion_raw)):
            self.rejected_count += 1
            return
        try:
            quaternion = normalize_quaternion(quaternion_raw)
        except ValueError:
            self.rejected_count += 1
            return

        now = time.monotonic()
        if self.first_time is None:
            self.first_time = now
            self._start_xy = (position[0], position[1])
        self.samples.append(PoseSample(now, position, quaternion))

        if self._start_xy is not None:
            displacement = math.hypot(
                position[0] - self._start_xy[0],
                position[1] - self._start_xy[1],
            )
            self.boundary_exceeded = (
                self.boundary_exceeded or displacement > self._max_displacement_m
            )


def collect_pose_samples(
    topic: str,
    duration_sec: float,
    wait_timeout_sec: float,
    position_scale: float,
    max_displacement_m: float,
    label: str,
) -> ViconPoseCollector:
    """지정 상태의 raw Vicon pose를 짧게 수집한다."""
    collector = ViconPoseCollector(topic, position_scale, max_displacement_m)
    wait_deadline = time.monotonic() + wait_timeout_sec
    print(f"[{label}] {duration_sec:.1f}초 측정 시작")

    while rclpy.ok():
        rclpy.spin_once(collector, timeout_sec=0.05)
        now = time.monotonic()
        if collector.first_time is None:
            if now >= wait_deadline:
                collector.destroy_node()
                raise RuntimeError(f"{topic}에서 raw Vicon pose를 받지 못했다")
            continue
        if collector.boundary_exceeded:
            print("안전 이동 반경 초과 — 즉시 에어를 차단하고 기체를 회수하세요.")
            break
        if now - collector.first_time >= duration_sec:
            break
    else:
        collector.destroy_node()
        raise RuntimeError("ROS 종료로 Vicon 수집을 완료하지 못했다")
    collector.collection_end_time = time.monotonic()
    return collector


def collection_stats(collector: ViconPoseCollector) -> FloatPoseStats:
    """수신이 멈춘 구간까지 포함하여 전체 수집 구간의 수신률을 보고한다."""
    stats = calculate_float_pose_stats(collector.samples)
    duration = collector.collection_end_time - collector.first_time
    return replace(stats, duration_sec=duration, sample_rate_hz=(stats.sample_count - 1) / duration)


def data_failures(
    stats: FloatPoseStats,
    collector: ViconPoseCollector,
    min_samples: int,
    min_rate_hz: float,
) -> list[str]:
    """Vicon 표본 자체의 품질 실패 원인을 반환한다."""
    failures: list[str] = []
    if stats.sample_count < min_samples:
        failures.append(f"표본 {stats.sample_count}개 < 최소 {min_samples}개")
    if stats.sample_rate_hz < min_rate_hz:
        failures.append(f"수신률 {stats.sample_rate_hz:.1f} Hz < 최소 {min_rate_hz:.1f} Hz")
    last_sample_age = collector.collection_end_time - collector.samples[-1].time_sec
    # 최소 수신률의 두 주기와 100ms 폴링 지터까지만 허용한다.
    if last_sample_age > max(0.1, 2.0 / min_rate_hz):
        failures.append(f"측정 종료 시 Vicon 미수신 {last_sample_age:.2f}초")
    if collector.rejected_count:
        failures.append(f"비정상 pose {collector.rejected_count}개 감지")
    if collector.boundary_exceeded:
        failures.append("부상 측정 중 안전 이동 반경 초과")
    return failures


def print_pose_stats(label: str, stats: FloatPoseStats) -> None:
    """자세 안정성과 XY 운동 통계를 출력한다."""
    print(f"[{label}] {stats.sample_count}개, {stats.sample_rate_hz:.1f} Hz")
    print(
        f"  body +Z 흔들림 RMS/max: {stats.body_z_rms_dispersion_deg:.4f}° / "
        f"{stats.body_z_max_dispersion_deg:.4f}°"
    )
    print(
        f"  XY 가속도: ({stats.acceleration_xy_m_s2[0]:+.6f}, "
        f"{stats.acceleration_xy_m_s2[1]:+.6f}) m/s²"
    )
    print(
        f"  최대 XY 이동: {stats.max_displacement_m:.4f} m, "
        f"Z 표준편차: {stats.position_std_m[2]:.6f} m"
    )


def write_report(path: Path, report: dict[str, Any]) -> None:
    """검사 리포트를 UTF-8 YAML로 저장한다."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(report, stream, allow_unicode=True, sort_keys=False)
    print(f"리포트 저장: {path}")


def run_check(args: argparse.Namespace) -> int:
    """한 실행에서 에어 OFF 기준과 에어 ON 부상 상태를 차례로 측정한다."""
    print("1/2: 에어를 끄고 기체를 고정한 뒤 Enter를 누르세요.")
    input()
    air_off_collector = collect_pose_samples(
        args.topic,
        args.air_off_duration,
        args.wait_timeout,
        args.position_scale,
        args.air_off_max_displacement,
        "에어 OFF",
    )
    try:
        air_off_stats = collection_stats(air_off_collector)
        failures = data_failures(
            air_off_stats,
            air_off_collector,
            args.min_samples,
            args.min_rate,
        )
    finally:
        air_off_collector.destroy_node()

    if failures:
        print_pose_stats("에어 OFF", air_off_stats)
        print("에어 OFF 기준 측정 실패 — 에어를 켜지 않습니다.")
        for failure in failures:
            print(f"  - {failure}")
        return 2

    print("2/2: 기체를 중앙에 두고 추력 OFF를 확인하세요.")
    print("사람이 회수 준비를 한 뒤 에어를 켜고 바로 Enter를 누르세요.")
    input()
    air_on_collector = collect_pose_samples(
        args.topic,
        args.air_on_duration,
        args.wait_timeout,
        args.position_scale,
        args.max_displacement,
        "에어 ON",
    )
    try:
        air_on_stats = collection_stats(air_on_collector)
        failures.extend(
            data_failures(
                air_on_stats,
                air_on_collector,
                args.min_samples,
                args.min_rate,
            )
        )
    finally:
        air_on_collector.destroy_node()

    trim_deg = trim_change_deg(
        air_off_stats.body_z_mean_world,
        air_on_stats.body_z_mean_world,
    )
    predicted_drift = (
        0.5 * air_on_stats.acceleration_magnitude_m_s2 * args.drift_horizon**2
    )
    slope_equivalent_deg = math.degrees(
        math.atan2(air_on_stats.acceleration_magnitude_m_s2, args.gravity)
    )

    print_pose_stats("에어 OFF", air_off_stats)
    print_pose_stats("에어 ON", air_on_stats)
    print(f"에어 OFF→ON body +Z 변화: {trim_deg:.4f}°")
    print(f"등가 경사: {slope_equivalent_deg:.5f}°")
    print(
        f"현재 가속도가 유지되면 정지 출발 {args.drift_horizon:.1f}초 후 "
        f"약 {predicted_drift:.3f} m 이동"
    )
    print("이 값은 보정값이 아니라 실제 부상 상태를 판단하기 위한 측정 결과입니다.")

    report: dict[str, Any] = {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "method": "vicon_air_off_on_comparison",
        "result": "pass" if not failures else "fail",
        "source_topic": args.topic,
        "air_off": float_stats_to_dict(air_off_stats),
        "air_on": float_stats_to_dict(air_on_stats),
        "trim_change_deg": trim_deg,
        "slope_equivalent_deg": slope_equivalent_deg,
        "predicted_drift_m": predicted_drift,
        "drift_horizon_sec": args.drift_horizon,
        "failures": failures,
        "note": "장착 quaternion을 생성하지 않으며 에어밸브도 자동 제어하지 않는다.",
    }
    if args.output is not None:
        write_report(args.output, report)

    if failures:
        print("측정 품질/안전 FAIL")
        for failure in failures:
            print(f"  - {failure}")
        return 2
    print("측정 완료 — 같은 압력과 적재 조건에서 여러 번 반복해 재현성을 확인하세요.")
    return 0


def build_parser() -> argparse.ArgumentParser:
    """명령행 파서를 구성한다."""
    parser = argparse.ArgumentParser(description="Vicon 기반 에어 OFF/ON 짧은 비교")
    # vicon_bridge_node가 기울기 게이팅 이전에 발행하는 raw 6-DOF 포즈 (이미 m 단위)
    parser.add_argument("--topic", required=True, help="실측 6-DOF PoseStamped 토픽 (FSS_VICON UDP는 평면 데이터만 제공)")
    parser.add_argument("--air-off-duration", type=float, default=3.0)
    parser.add_argument("--air-on-duration", type=float, default=5.0)
    parser.add_argument("--wait-timeout", type=float, default=10.0)
    parser.add_argument("--position-scale", type=float, default=1.0)
    parser.add_argument("--min-samples", type=int, default=30)
    parser.add_argument("--min-rate", type=float, default=20.0)
    parser.add_argument("--air-off-max-displacement", type=float, default=0.01)
    parser.add_argument("--max-displacement", type=float, default=0.20)
    parser.add_argument("--drift-horizon", type=float, default=10.0)
    parser.add_argument("--gravity", type=float, default=9.80665)
    parser.add_argument("--output", type=Path, help="선택적 YAML 리포트 경로")
    return parser


def validate_args(args: argparse.Namespace) -> None:
    """시간·스케일·안전 반경이 양수인지 검사한다."""
    values = (
        args.air_off_duration,
        args.air_on_duration,
        args.wait_timeout,
        args.position_scale,
        args.min_rate,
        args.air_off_max_displacement,
        args.max_displacement,
        args.drift_horizon,
        args.gravity,
    )
    if not all(math.isfinite(value) and value > 0.0 for value in values):
        raise ValueError("시간·스케일·안전 반경은 양수 유한값이어야 한다")
    if args.min_samples < 3:
        raise ValueError("--min-samples는 3 이상이어야 한다")


def main() -> int:
    """CLI 메인 엔트리포인트."""
    args = build_parser().parse_args()
    try:
        validate_args(args)
    except ValueError as error:
        print(f"오류: {error}", file=sys.stderr)
        return 3

    rclpy.init(args=[])
    try:
        return run_check(args)
    except (EOFError, KeyboardInterrupt, OSError, RuntimeError, ValueError) as error:
        print(f"오류: {error}", file=sys.stderr)
        return 3
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
