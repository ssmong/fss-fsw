"""센서 노이즈 주입 모델 — Vicon + IMU 시뮬레이션.

실제 상태에 데이터시트 기반 노이즈를 주입하여 센서 출력을 시뮬레이션한다.
노이즈 파라미터는 ekf.yaml과 동일한 값을 사용하여 일관성을 유지한다.

Vicon 모델:
  - 위치: N(0, σ²_pos)   (MX T10: 0.5mm)
  - 방향: N(0, σ²_hdg)   (MX T10: ~0.1°)

IMU 모델:
  - 자이로: true_ω + bias + N(0, σ²_arw × fs)
  - 가속도: true_a + bias + N(0, σ²_vrw × fs)
  - 바이어스: 느린 랜덤 워크 (σ_brw × √dt)
"""

import numpy as np
from numpy.typing import NDArray


class ViconModel:
    """Vicon MX T10 센서 노이즈 모델."""

    def __init__(
        self,
        position_noise: float = 5e-4,
        heading_noise: float = 1.745e-3,
    ) -> None:
        """초기화.

        Args:
            position_noise: 위치 노이즈 σ [m]
            heading_noise: 방향 노이즈 σ [rad]
        """
        self._pos_std = position_noise
        self._hdg_std = heading_noise

    def measure(self, true_state: NDArray[np.float64]) -> tuple[float, float, float]:
        """Vicon 측정값을 생성한다.

        Args:
            true_state: [px, py, θ, vx, vy, ω]

        Returns:
            (x_meas, y_meas, θ_meas) 노이즈 포함
        """
        x = true_state[0] + np.random.normal(0, self._pos_std)
        y = true_state[1] + np.random.normal(0, self._pos_std)
        th = true_state[2] + np.random.normal(0, self._hdg_std)
        return x, y, th


class IMUModel:
    """Xsens MTi-630-DK IMU 노이즈 + 바이어스 모델."""

    def __init__(
        self,
        gyro_noise_density: float = 1.2217e-4,
        accel_noise_density: float = 5.886e-4,
        gyro_bias_random_walk: float = 1e-5,
        accel_bias_random_walk: float = 1e-4,
        sample_rate_hz: float = 100.0,
    ) -> None:
        """초기화.

        Args:
            gyro_noise_density: ARW [rad/s/√Hz]
            accel_noise_density: VRW [m/s²/√Hz]
            gyro_bias_random_walk: 자이로 bias RW [rad/s/√s]
            accel_bias_random_walk: 가속도 bias RW [m/s²/√s]
            sample_rate_hz: 샘플링 주파수 [Hz]
        """
        self._gyro_std = gyro_noise_density * np.sqrt(sample_rate_hz)
        self._accel_std = accel_noise_density * np.sqrt(sample_rate_hz)
        self._gyro_brw = gyro_bias_random_walk
        self._accel_brw = accel_bias_random_walk

        # 바이어스 상태 (느린 랜덤 워크)
        self.gyro_bias = 0.0
        self.accel_bias_x = 0.0
        self.accel_bias_y = 0.0

    def measure(
        self, true_state: NDArray[np.float64], wrench: NDArray[np.float64],
        mass: float, dt: float,
    ) -> tuple[float, float, float]:
        """IMU 측정값을 생성한다.

        Args:
            true_state: [px, py, θ, vx, vy, ω]
            wrench: [Fx, Fy, Mz] 본체 프레임
            mass: 질량 [kg]
            dt: 시간 간격 [s]

        Returns:
            (omega_z, ax, ay) 바이어스 + 노이즈 포함
        """
        # 바이어스 랜덤 워크 갱신
        sqrt_dt = np.sqrt(dt)
        self.gyro_bias += np.random.normal(0, self._gyro_brw * sqrt_dt)
        self.accel_bias_x += np.random.normal(0, self._accel_brw * sqrt_dt)
        self.accel_bias_y += np.random.normal(0, self._accel_brw * sqrt_dt)

        # 실제 값 + 바이어스 + 백색 노이즈
        omega_z = true_state[5] + self.gyro_bias + np.random.normal(0, self._gyro_std)
        ax = wrench[0] / mass + self.accel_bias_x + np.random.normal(0, self._accel_std)
        ay = wrench[1] / mass + self.accel_bias_y + np.random.normal(0, self._accel_std)

        return omega_z, ax, ay
