"""IMU 장착 회전과 정지 품질 판정 단위 테스트."""

import math

import pytest

from fss_sensors.imu_calibration import (
    StationaryLimits,
    calculate_stationary_stats,
    evaluate_stationary,
)
from fss_sensors.float_calibration import (
    PoseSample,
    calculate_float_pose_stats,
    fit_constant_acceleration,
    trim_change_deg,
)
from fss_sensors.imu_transform import (
    quaternion_conjugate,
    quaternion_multiply,
    quaternion_to_matrix,
    rotate_covariance,
    rotate_vector,
    yaw_quaternion,
)


def test_yaw_rotation_maps_imu_x_to_body_y() -> None:
    """+90° 센서→본체 yaw가 X축을 Y축으로 보낸다."""
    rotation = quaternion_to_matrix(yaw_quaternion(math.pi / 2.0))
    result = rotate_vector((1.0, 0.0, 0.0), rotation)
    assert result == pytest.approx((0.0, 1.0, 0.0), abs=1e-12)


def test_covariance_rotates_with_axes() -> None:
    """90° yaw에서 X/Y 축 공분산이 서로 교환된다."""
    rotation = quaternion_to_matrix(yaw_quaternion(math.pi / 2.0))
    covariance = rotate_covariance([1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0], rotation)
    assert covariance == pytest.approx([2.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 3.0])


def test_orientation_mounting_composition_returns_body_identity() -> None:
    """world→IMU 자세가 장착 회전과 같으면 world→body 자세는 identity다."""
    mounting = yaw_quaternion(0.4)
    world_from_body = quaternion_multiply(mounting, quaternion_conjugate(mounting))
    assert world_from_body == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1e-12)


def test_stationary_quality_passes_clean_body_data() -> None:
    """정상적인 정지 body-frame 표본은 품질 검사를 통과한다."""
    gyro = [(0.001, -0.001, 0.002)] * 100
    accel = [(0.01, -0.02, 9.8066)] * 100
    stats = calculate_stationary_stats(gyro, accel)
    failures = evaluate_stationary(
        stats,
        StationaryLimits(),
    )
    assert failures == []


def test_stationary_quality_does_not_treat_horizontal_mean_as_mounting_error() -> None:
    """받침·테이블 경사가 섞인 수평 평균으로 장착각을 판정하지 않는다."""
    gyro = [(0.0, 0.0, 0.0)] * 100
    accel = [(0.3, 0.0, 9.802)] * 100
    stats = calculate_stationary_stats(gyro, accel)
    failures = evaluate_stationary(stats, StationaryLimits())
    assert failures == []


def test_constant_acceleration_fit_recovers_quadratic_motion() -> None:
    """Vicon 위치의 2차 피팅에서 실제 일정 가속도를 복원한다."""
    times = [index * 0.05 for index in range(101)]
    acceleration = 0.012
    positions = [1.2 + 0.03 * time + 0.5 * acceleration * time * time for time in times]
    assert fit_constant_acceleration(times, positions) == pytest.approx(acceleration, abs=1e-12)


def test_float_stats_report_drift_and_trim_change() -> None:
    """부상 표본에서 XY 가속도와 body +Z trim 변화를 계산한다."""
    acceleration = 0.01
    tilt_rad = math.radians(0.4)
    tilted_about_x = (math.sin(tilt_rad / 2.0), 0.0, 0.0, math.cos(tilt_rad / 2.0))
    samples = [
        PoseSample(
            time_sec=index * 0.05,
            position_m=(0.5 * acceleration * (index * 0.05) ** 2, 0.0, 0.1),
            orientation_xyzw=tilted_about_x,
        )
        for index in range(101)
    ]
    stats = calculate_float_pose_stats(samples)
    assert stats.acceleration_xy_m_s2 == pytest.approx((acceleration, 0.0), abs=1e-12)
    assert trim_change_deg((0.0, 0.0, 1.0), stats.body_z_mean_world) == pytest.approx(
        0.4, abs=1e-10
    )
