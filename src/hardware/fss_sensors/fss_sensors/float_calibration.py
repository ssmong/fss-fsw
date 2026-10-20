"""Vicon 6-DOF 표본으로 부상 trim과 무추력 드리프트를 계산한다."""

import math
import statistics
from dataclasses import dataclass
from typing import Any

from fss_sensors.imu_transform import Quaternion, Vector3, quaternion_to_matrix


@dataclass(frozen=True)
class PoseSample:
    """단일 Vicon pose 표본."""

    time_sec: float
    position_m: Vector3
    orientation_xyzw: Quaternion


@dataclass(frozen=True)
class FloatPoseStats:
    """한 구간의 자세 안정성과 XY 드리프트 통계."""

    sample_count: int
    duration_sec: float
    sample_rate_hz: float
    position_mean_m: Vector3
    position_std_m: Vector3
    body_z_mean_world: Vector3
    body_z_rms_dispersion_deg: float
    body_z_max_dispersion_deg: float
    acceleration_xy_m_s2: tuple[float, float]
    acceleration_magnitude_m_s2: float
    max_displacement_m: float


def _mean_vector(vectors: list[Vector3]) -> Vector3:
    """3차원 벡터의 축별 평균을 계산한다."""
    return (
        statistics.fmean(vector[0] for vector in vectors),
        statistics.fmean(vector[1] for vector in vectors),
        statistics.fmean(vector[2] for vector in vectors),
    )


def _std_vector(vectors: list[Vector3]) -> Vector3:
    """3차원 벡터의 축별 모집단 표준편차를 계산한다."""
    return (
        statistics.pstdev(vector[0] for vector in vectors),
        statistics.pstdev(vector[1] for vector in vectors),
        statistics.pstdev(vector[2] for vector in vectors),
    )


def _normalize_vector(vector: Vector3) -> Vector3:
    """3차원 벡터를 단위 벡터로 정규화한다."""
    norm = math.sqrt(sum(value * value for value in vector))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("평균 방향 벡터를 정규화할 수 없다")
    return (vector[0] / norm, vector[1] / norm, vector[2] / norm)


def _angle_deg(left: Vector3, right: Vector3) -> float:
    """두 단위 방향 벡터 사이 각도를 도 단위로 반환한다."""
    dot = max(-1.0, min(1.0, sum(a * b for a, b in zip(left, right, strict=True))))
    return math.degrees(math.acos(dot))


def _solve_3x3(matrix: list[list[float]], vector: list[float]) -> tuple[float, float, float]:
    """부분 피벗 Gaussian elimination으로 3×3 선형계를 푼다."""
    augmented = [matrix[row][:] + [vector[row]] for row in range(3)]
    for pivot_col in range(3):
        pivot_row = max(range(pivot_col, 3), key=lambda row: abs(augmented[row][pivot_col]))
        if abs(augmented[pivot_row][pivot_col]) < 1e-12:
            raise ValueError("가속도 피팅 행렬이 특이하다")
        augmented[pivot_col], augmented[pivot_row] = (
            augmented[pivot_row],
            augmented[pivot_col],
        )
        pivot = augmented[pivot_col][pivot_col]
        augmented[pivot_col] = [value / pivot for value in augmented[pivot_col]]
        for row in range(3):
            if row == pivot_col:
                continue
            factor = augmented[row][pivot_col]
            augmented[row] = [
                augmented[row][col] - factor * augmented[pivot_col][col]
                for col in range(4)
            ]
    return (augmented[0][3], augmented[1][3], augmented[2][3])


def fit_constant_acceleration(times_sec: list[float], positions_m: list[float]) -> float:
    """위치 표본에 ``p=p0+v0*t+0.5*a*t²``를 최소제곱 피팅한다."""
    if len(times_sec) != len(positions_m) or len(times_sec) < 3:
        raise ValueError("가속도 피팅에는 같은 길이의 표본 3개 이상이 필요하다")

    start = times_sec[0]
    duration = times_sec[-1] - start
    if not math.isfinite(duration) or duration <= 1e-6:
        raise ValueError("가속도 피팅 시간 구간이 너무 짧다")

    normalized_times = [(time_value - start) / duration for time_value in times_sec]
    rows = [(1.0, value, value * value) for value in normalized_times]
    normal_matrix = [
        [sum(row[i] * row[j] for row in rows) for j in range(3)] for i in range(3)
    ]
    normal_vector = [
        sum(row[i] * position for row, position in zip(rows, positions_m, strict=True))
        for i in range(3)
    ]
    _, _, quadratic_coefficient = _solve_3x3(normal_matrix, normal_vector)
    return 2.0 * quadratic_coefficient / (duration * duration)


def calculate_float_pose_stats(samples: list[PoseSample]) -> FloatPoseStats:
    """Vicon pose 구간에서 부상 자세와 XY 가속도 통계를 계산한다."""
    if len(samples) < 3:
        raise ValueError("Vicon pose 표본이 3개 미만이다")

    times = [sample.time_sec for sample in samples]
    if any(times[index + 1] <= times[index] for index in range(len(times) - 1)):
        raise ValueError("Vicon 표본 시간이 단조 증가하지 않는다")

    positions = [sample.position_m for sample in samples]
    body_z_vectors = []
    for sample in samples:
        rotation = quaternion_to_matrix(sample.orientation_xyzw)
        body_z_vectors.append((rotation[0][2], rotation[1][2], rotation[2][2]))

    body_z_mean = _normalize_vector(_mean_vector(body_z_vectors))
    dispersions = [_angle_deg(vector, body_z_mean) for vector in body_z_vectors]
    acceleration_x = fit_constant_acceleration(times, [value[0] for value in positions])
    acceleration_y = fit_constant_acceleration(times, [value[1] for value in positions])
    start_x, start_y, _ = positions[0]
    max_displacement = max(
        math.hypot(position[0] - start_x, position[1] - start_y) for position in positions
    )
    duration = times[-1] - times[0]
    return FloatPoseStats(
        sample_count=len(samples),
        duration_sec=duration,
        sample_rate_hz=(len(samples) - 1) / duration,
        position_mean_m=_mean_vector(positions),
        position_std_m=_std_vector(positions),
        body_z_mean_world=body_z_mean,
        body_z_rms_dispersion_deg=math.sqrt(
            statistics.fmean(value * value for value in dispersions)
        ),
        body_z_max_dispersion_deg=max(dispersions),
        acceleration_xy_m_s2=(acceleration_x, acceleration_y),
        acceleration_magnitude_m_s2=math.hypot(acceleration_x, acceleration_y),
        max_displacement_m=max_displacement,
    )


def trim_change_deg(reference_body_z: Vector3, floated_body_z: Vector3) -> float:
    """에어 OFF/ON 평균 body +Z 방향 사이의 총 trim 변화를 반환한다."""
    return _angle_deg(_normalize_vector(reference_body_z), _normalize_vector(floated_body_z))


def float_stats_to_dict(stats: FloatPoseStats) -> dict[str, Any]:
    """YAML 직렬화가 쉬운 부상 통계 사전을 만든다."""
    return {
        "sample_count": stats.sample_count,
        "duration_sec": stats.duration_sec,
        "sample_rate_hz": stats.sample_rate_hz,
        "position_mean_m": list(stats.position_mean_m),
        "position_std_m": list(stats.position_std_m),
        "body_z_mean_world": list(stats.body_z_mean_world),
        "body_z_rms_dispersion_deg": stats.body_z_rms_dispersion_deg,
        "body_z_max_dispersion_deg": stats.body_z_max_dispersion_deg,
        "acceleration_xy_m_s2": list(stats.acceleration_xy_m_s2),
        "acceleration_magnitude_m_s2": stats.acceleration_magnitude_m_s2,
        "max_displacement_m": stats.max_displacement_m,
    }
