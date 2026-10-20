"""IMU 센서 프레임을 FSS 본체 프레임으로 변환하는 순수 수학 유틸리티."""

import math
from collections.abc import Sequence

Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]
Matrix3 = tuple[Vector3, Vector3, Vector3]


def normalize_quaternion(quaternion: Sequence[float]) -> Quaternion:
    """쿼터니언을 정규화한다.

    Args:
        quaternion: ``[x, y, z, w]`` 순서 쿼터니언.

    Returns:
        정규화된 ``(x, y, z, w)`` 쿼터니언.

    Raises:
        ValueError: 원소 수가 4가 아니거나 크기가 0인 경우.
    """
    if len(quaternion) != 4:
        raise ValueError("쿼터니언은 [x, y, z, w] 4개 값이어야 한다")

    values = tuple(float(value) for value in quaternion)
    norm = math.sqrt(sum(value * value for value in values))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("쿼터니언 크기가 0이거나 유한하지 않다")
    return (values[0] / norm, values[1] / norm, values[2] / norm, values[3] / norm)


def quaternion_conjugate(quaternion: Sequence[float]) -> Quaternion:
    """단위 쿼터니언의 켤레를 반환한다."""
    x, y, z, w = normalize_quaternion(quaternion)
    return (-x, -y, -z, w)


def quaternion_multiply(left: Sequence[float], right: Sequence[float]) -> Quaternion:
    """두 쿼터니언의 Hamilton 곱을 반환한다."""
    lx, ly, lz, lw = normalize_quaternion(left)
    rx, ry, rz, rw = normalize_quaternion(right)
    return normalize_quaternion(
        (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )
    )


def quaternion_to_matrix(quaternion: Sequence[float]) -> Matrix3:
    """``[x, y, z, w]`` 쿼터니언을 3×3 회전행렬로 변환한다."""
    x, y, z, w = normalize_quaternion(quaternion)
    return (
        (
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ),
        (
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ),
        (
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ),
    )


def rotate_vector(vector: Sequence[float], rotation: Matrix3) -> Vector3:
    """3차원 벡터를 회전행렬로 변환한다."""
    if len(vector) != 3:
        raise ValueError("벡터는 3개 값이어야 한다")
    x, y, z = (float(value) for value in vector)
    return (
        rotation[0][0] * x + rotation[0][1] * y + rotation[0][2] * z,
        rotation[1][0] * x + rotation[1][1] * y + rotation[1][2] * z,
        rotation[2][0] * x + rotation[2][1] * y + rotation[2][2] * z,
    )


def rotate_covariance(covariance: Sequence[float], rotation: Matrix3) -> list[float]:
    """행 우선 3×3 공분산에 ``R C Rᵀ``를 적용한다.

    ROS의 미제공 표식인 첫 원소 ``-1``은 그대로 보존한다.
    """
    if len(covariance) != 9:
        raise ValueError("공분산은 행 우선 3×3 값 9개여야 한다")
    if covariance[0] < 0.0:
        return [float(value) for value in covariance]

    source = [[float(covariance[row * 3 + col]) for col in range(3)] for row in range(3)]
    rotated = [[0.0] * 3 for _ in range(3)]
    for row in range(3):
        for col in range(3):
            rotated[row][col] = sum(
                rotation[row][i] * source[i][j] * rotation[col][j]
                for i in range(3)
                for j in range(3)
            )
    return [rotated[row][col] for row in range(3) for col in range(3)]


def yaw_quaternion(yaw_rad: float) -> Quaternion:
    """+Z축 회전 쿼터니언을 반환한다."""
    half_yaw = 0.5 * yaw_rad
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


def quaternion_to_rpy(quaternion: Sequence[float]) -> Vector3:
    """쿼터니언을 ROS 표준 roll, pitch, yaw로 변환한다."""
    x, y, z, w = normalize_quaternion(quaternion)

    sin_roll_cos_pitch = 2.0 * (w * x + y * z)
    cos_roll_cos_pitch = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll_cos_pitch, cos_roll_cos_pitch)

    sin_pitch = 2.0 * (w * y - z * x)
    pitch = (
        math.copysign(math.pi / 2.0, sin_pitch)
        if abs(sin_pitch) >= 1.0
        else math.asin(sin_pitch)
    )

    sin_yaw_cos_pitch = 2.0 * (w * z + x * y)
    cos_yaw_cos_pitch = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch)
    return (roll, pitch, yaw)
