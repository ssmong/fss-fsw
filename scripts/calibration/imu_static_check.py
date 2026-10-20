#!/usr/bin/env python3
"""에어 OFF·기체 고정 상태에서 body-frame IMU 품질을 검사한다."""

import argparse
import math
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

# colcon 소싱 없이도 저장소 소스 트리에서 순수 계산 모듈을 찾는다.
REPO_ROOT = Path(__file__).resolve().parents[2]
SENSOR_PACKAGE = REPO_ROOT / "src" / "hardware" / "fss_sensors"
sys.path.insert(0, str(SENSOR_PACKAGE))

from fss_sensors.imu_calibration import (  # noqa: E402
    StationaryLimits,
    StationaryStats,
    calculate_stationary_stats,
    evaluate_stationary,
    stats_to_dict,
)
from fss_sensors.imu_transform import Vector3  # noqa: E402


class IMUSampleCollector(Node):
    """유한한 body-frame IMU 표본을 수집한다."""

    def __init__(self, topic: str) -> None:
        """수집 노드를 초기화한다."""
        super().__init__("imu_static_check")
        self.gyro_samples: list[Vector3] = []
        self.accel_samples: list[Vector3] = []
        self.first_sample_time: float | None = None
        self.last_sample_time: float | None = None
        self.collection_end_time: float | None = None
        self.rejected_count = 0
        self._subscription = self.create_subscription(
            Imu,
            topic,
            self._on_imu,
            qos_profile_sensor_data,
        )

    def _on_imu(self, msg: Imu) -> None:
        """제공된 유한한 6축 표본만 저장한다."""
        gyro = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        accel = (
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        )
        availability = (msg.angular_velocity_covariance[0], msg.linear_acceleration_covariance[0])
        if any(value < 0.0 for value in availability) or not all(
            math.isfinite(value) for value in (*gyro, *accel, *availability)
        ):
            self.rejected_count += 1
            return

        now = time.monotonic()
        if self.first_sample_time is None:
            self.first_sample_time = now
        self.last_sample_time = now
        self.gyro_samples.append(gyro)
        self.accel_samples.append(accel)


def collect_samples(topic: str, duration_sec: float, wait_timeout_sec: float) -> IMUSampleCollector:
    """첫 메시지부터 지정 시간 동안 IMU 표본을 수집한다."""
    collector = IMUSampleCollector(topic)
    wait_deadline = time.monotonic() + wait_timeout_sec
    print(f"[{topic}] 에어 OFF·기체 고정 상태로 {duration_sec:.1f}초 기다리세요.")

    while rclpy.ok():
        rclpy.spin_once(collector, timeout_sec=0.1)
        now = time.monotonic()
        if collector.first_sample_time is None:
            if now >= wait_deadline:
                collector.destroy_node()
                raise RuntimeError(f"{topic}에서 {wait_timeout_sec:.1f}초 동안 IMU를 받지 못했다")
            continue
        if now - collector.first_sample_time >= duration_sec:
            break
    else:
        collector.destroy_node()
        raise RuntimeError("ROS 종료로 IMU 수집을 완료하지 못했다")
    collector.collection_end_time = time.monotonic()
    return collector


def sample_rate_hz(collector: IMUSampleCollector) -> float:
    """수집 표본의 관측 주파수를 계산한다."""
    if (
        len(collector.gyro_samples) < 2
        or collector.first_sample_time is None
        or collector.last_sample_time is None
        or collector.collection_end_time is None
    ):
        return 0.0
    span = collector.collection_end_time - collector.first_sample_time
    return (len(collector.gyro_samples) - 1) / span if span > 0.0 else 0.0


def print_stats(stats: StationaryStats, rate_hz: float) -> None:
    """정지 통계와 수평축 진단값을 출력한다."""
    horizontal = math.hypot(stats.accel_mean[0], stats.accel_mean[1])
    print(f"표본: {stats.sample_count}개 ({rate_hz:.1f} Hz)")
    print(
        "gyro 평균 [rad/s]: "
        + ", ".join(f"{value:+.6f}" for value in stats.gyro_mean)
    )
    print(
        "gyro 표준편차   : "
        + ", ".join(f"{value:.6f}" for value in stats.gyro_std)
    )
    print(
        "accel 평균 [m/s²]: "
        + ", ".join(f"{value:+.6f}" for value in stats.accel_mean)
    )
    print(
        "accel 표준편차   : "
        + ", ".join(f"{value:.6f}" for value in stats.accel_std)
    )
    print(f"|accel|: {stats.accel_norm_mean:.6f} ± {stats.accel_norm_std:.6f} m/s²")
    print(f"body XY 평균: {horizontal:.6f} m/s² (표시만 하며 장착각 자동 보정에는 미사용)")


def write_report(path: Path, report: dict[str, Any]) -> None:
    """검사 리포트를 UTF-8 YAML로 저장한다."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(report, stream, allow_unicode=True, sort_keys=False)
    print(f"리포트 저장: {path}")


def run_check(args: argparse.Namespace) -> int:
    """정지 표본을 수집하고 센서 품질을 판정한다."""
    collector = collect_samples(args.topic, args.duration, args.wait_timeout)
    try:
        stats = calculate_stationary_stats(collector.gyro_samples, collector.accel_samples)
        rate_hz = sample_rate_hz(collector)
        last_sample_age = collector.collection_end_time - collector.last_sample_time
        rejected_count = collector.rejected_count
    finally:
        collector.destroy_node()

    failures = evaluate_stationary(
        stats,
        StationaryLimits(
            gravity_m_s2=args.gravity,
            gravity_tolerance_m_s2=args.gravity_tolerance,
            max_gyro_mean_rad_s=args.max_gyro_mean,
            max_gyro_std_rad_s=args.max_gyro_std,
            max_accel_std_m_s2=args.max_accel_std,
        ),
    )
    if stats.sample_count < args.min_samples:
        failures.append(f"표본 {stats.sample_count}개 < 최소 {args.min_samples}개")
    if rate_hz < args.min_rate:
        failures.append(f"수신률 {rate_hz:.1f} Hz < 최소 {args.min_rate:.1f} Hz")
    # 최소 수신률의 두 주기와 100ms 폴링 지터까지만 허용한다.
    if last_sample_age > max(0.1, 2.0 / args.min_rate):
        failures.append(f"측정 종료 시 IMU 미수신 {last_sample_age:.2f}초")
    if rejected_count:
        failures.append(f"비정상 또는 필수 측정 미제공 표본 {rejected_count}개 감지")

    print_stats(stats, rate_hz)
    if failures:
        print("결과: FAIL")
        for failure in failures:
            print(f"  - {failure}")
    else:
        print("결과: PASS — 정적 센서 품질이 정상입니다.")

    report: dict[str, Any] = {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "method": "air_off_static_check",
        "result": "pass" if not failures else "fail",
        "source_topic": args.topic,
        "sample_rate_hz": rate_hz,
        "last_sample_age_sec": last_sample_age,
        "rejected_count": rejected_count,
        "body_frame_stats": stats_to_dict(stats),
        "horizontal_accel_diagnostic_m_s2": math.hypot(
            stats.accel_mean[0], stats.accel_mean[1]
        ),
        "failures": failures,
        "note": "수평 평균은 받침과 테이블 경사를 포함하므로 장착각으로 사용하지 않는다.",
    }
    if args.output is not None:
        write_report(args.output, report)
    return 0 if not failures else 2


def build_parser() -> argparse.ArgumentParser:
    """명령행 파서를 구성한다."""
    parser = argparse.ArgumentParser(description="에어 OFF body-frame IMU 정적 품질 검사")
    parser.add_argument("--topic", default="/sensors/imu")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--wait-timeout", type=float, default=10.0)
    parser.add_argument("--min-samples", type=int, default=100)
    parser.add_argument("--min-rate", type=float, default=20.0)
    parser.add_argument("--gravity", type=float, default=9.80665)
    parser.add_argument("--gravity-tolerance", type=float, default=0.30)
    parser.add_argument("--max-gyro-mean", type=float, default=0.05)
    parser.add_argument("--max-gyro-std", type=float, default=0.01)
    parser.add_argument("--max-accel-std", type=float, default=0.10)
    parser.add_argument("--output", type=Path, help="선택적 YAML 리포트 경로")
    return parser


def validate_args(args: argparse.Namespace) -> None:
    """시간과 품질 임계값이 양수인지 검사한다."""
    values = (
        args.duration,
        args.wait_timeout,
        args.min_rate,
        args.gravity,
        args.gravity_tolerance,
        args.max_gyro_mean,
        args.max_gyro_std,
        args.max_accel_std,
    )
    if not all(math.isfinite(value) and value > 0.0 for value in values):
        raise ValueError("시간·수신률·임계값은 양수 유한값이어야 한다")
    if args.min_samples <= 0:
        raise ValueError("--min-samples는 양의 정수여야 한다")


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
    except (KeyboardInterrupt, OSError, RuntimeError, ValueError) as error:
        print(f"오류: {error}", file=sys.stderr)
        return 3
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
