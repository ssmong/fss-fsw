"""IMU 정지 수집 통계와 품질 판정 유틸리티."""

import math
import statistics
from dataclasses import dataclass
from typing import Any

from fss_sensors.imu_transform import Matrix3, Vector3, rotate_vector


@dataclass(frozen=True)
class StationaryStats:
    """정지 구간 IMU 통계."""

    sample_count: int
    gyro_mean: Vector3
    gyro_std: Vector3
    accel_mean: Vector3
    accel_std: Vector3
    accel_norm_mean: float
    accel_norm_std: float


@dataclass(frozen=True)
class StationaryLimits:
    """정지 품질 합격 기준."""

    gravity_m_s2: float = 9.80665
    gravity_tolerance_m_s2: float = 0.30
    max_gyro_mean_rad_s: float = 0.05
    max_gyro_std_rad_s: float = 0.01
    max_accel_std_m_s2: float = 0.10


def _vector_mean(samples: list[Vector3]) -> Vector3:
    """3축 표본 평균을 계산한다."""
    return (
        statistics.fmean(sample[0] for sample in samples),
        statistics.fmean(sample[1] for sample in samples),
        statistics.fmean(sample[2] for sample in samples),
    )


def _vector_std(samples: list[Vector3]) -> Vector3:
    """3축 모집단 표준편차를 계산한다."""
    return (
        statistics.pstdev(sample[0] for sample in samples),
        statistics.pstdev(sample[1] for sample in samples),
        statistics.pstdev(sample[2] for sample in samples),
    )


def calculate_stationary_stats(
    gyro_samples: list[Vector3],
    accel_samples: list[Vector3],
    rotation: Matrix3 | None = None,
) -> StationaryStats:
    """동일 시각에 수집한 gyro/accel 표본 통계를 계산한다.

    Args:
        gyro_samples: 센서 또는 본체 프레임 각속도 표본.
        accel_samples: 센서 또는 본체 프레임 가속도 표본.
        rotation: 표본에 먼저 적용할 선택적 3×3 회전행렬.

    Returns:
        지정 프레임에서 계산한 정지 통계.

    Raises:
        ValueError: 표본이 없거나 두 표본 수가 다른 경우.
    """
    if not gyro_samples or not accel_samples:
        raise ValueError("IMU 표본이 비어 있다")
    if len(gyro_samples) != len(accel_samples):
        raise ValueError("gyro와 accel 표본 수가 다르다")

    if rotation is not None:
        gyro_samples = [rotate_vector(sample, rotation) for sample in gyro_samples]
        accel_samples = [rotate_vector(sample, rotation) for sample in accel_samples]

    accel_norms = [math.sqrt(sum(value * value for value in sample)) for sample in accel_samples]
    return StationaryStats(
        sample_count=len(gyro_samples),
        gyro_mean=_vector_mean(gyro_samples),
        gyro_std=_vector_std(gyro_samples),
        accel_mean=_vector_mean(accel_samples),
        accel_std=_vector_std(accel_samples),
        accel_norm_mean=statistics.fmean(accel_norms),
        accel_norm_std=statistics.pstdev(accel_norms),
    )


def evaluate_stationary(stats: StationaryStats, limits: StationaryLimits) -> list[str]:
    """정지 통계가 기준을 벗어난 이유를 반환한다.

    빈 리스트면 모든 기준을 통과한 것이다.
    """
    failures: list[str] = []
    gravity_error = abs(stats.accel_norm_mean - limits.gravity_m_s2)
    if gravity_error > limits.gravity_tolerance_m_s2:
        failures.append(
            f"가속도 크기 오차 {gravity_error:.3f} m/s² > "
            f"{limits.gravity_tolerance_m_s2:.3f} m/s²"
        )

    gyro_mean_max = max(abs(value) for value in stats.gyro_mean)
    if gyro_mean_max > limits.max_gyro_mean_rad_s:
        failures.append(
            f"자이로 평균 최대값 {gyro_mean_max:.4f} rad/s > "
            f"{limits.max_gyro_mean_rad_s:.4f} rad/s"
        )

    gyro_std_max = max(stats.gyro_std)
    if gyro_std_max > limits.max_gyro_std_rad_s:
        failures.append(
            f"자이로 표준편차 최대값 {gyro_std_max:.4f} rad/s > "
            f"{limits.max_gyro_std_rad_s:.4f} rad/s"
        )

    accel_std_max = max(stats.accel_std)
    if accel_std_max > limits.max_accel_std_m_s2:
        failures.append(
            f"가속도 표준편차 최대값 {accel_std_max:.4f} m/s² > "
            f"{limits.max_accel_std_m_s2:.4f} m/s²"
        )

    return failures


def stats_to_dict(stats: StationaryStats) -> dict[str, Any]:
    """YAML 직렬화가 쉬운 정지 통계 사전을 만든다."""
    return {
        "sample_count": stats.sample_count,
        "gyro_mean_rad_s": list(stats.gyro_mean),
        "gyro_std_rad_s": list(stats.gyro_std),
        "accel_mean_m_s2": list(stats.accel_mean),
        "accel_std_m_s2": list(stats.accel_std),
        "accel_norm_mean_m_s2": stats.accel_norm_mean,
        "accel_norm_std_m_s2": stats.accel_norm_std,
    }
