"""통합 유도 관리자 (GuidanceManager).

target(단일 목표점) 또는 trajectory(시간 기반 경로) 모드를 지원한다.
Hydra instantiate와 호환되도록 설계되었다.

사용 예:
  guidance = GuidanceManager(mode="target", target={"x": 0, "y": 0, "theta": 0})
  ref = guidance.compute(state, t)  # → TrajectoryReference 데이터
"""

import logging
import math

import numpy as np

logger = logging.getLogger(__name__)


class GuidanceManager:
    """통합 유도 관리자 — target 또는 trajectory 모드.

    Hydra config에서 직접 instantiate 가능:
      _target_: fss_gnc.guidance.GuidanceManager
      mode: "target"
      target: {x: 0.0, y: 0.0, theta: 0.0}

    Attributes:
        mode: 현재 유도 모드 ("target" 또는 "trajectory")
    """

    def __init__(
        self,
        mode: str = "target",
        target: dict | None = None,
        trajectory_points: list[dict] | None = None,
        tolerance_position: float = 0.01,
        tolerance_angle: float = 0.02,
        max_speed: float = 0.1,
        max_angular_rate: float = 0.2,
    ) -> None:
        """유도 관리자 초기화.

        Args:
            mode: "target" (단일 목표점) 또는 "trajectory" (시간 기반 경로)
            target: 목표점 {x, y, theta} (target 모드)
            trajectory_points: 궤적 점 목록 [{t, x, y, theta}, ...] (trajectory 모드)
            tolerance_position: 위치 수렴 허용 오차 [m]
            tolerance_angle: 각도 수렴 허용 오차 [rad]
            max_speed: 최대 접근 속도 [m/s]
            max_angular_rate: 최대 각속도 [rad/s]
        """
        self.mode = mode
        self._tol_pos = tolerance_position
        self._tol_ang = tolerance_angle
        self._max_speed = max_speed
        self._max_omega = max_angular_rate

        # Target 모드 기본값
        self._target_x = 0.0
        self._target_y = 0.0
        self._target_theta = 0.0
        if target:
            self._target_x = target["x"]
            self._target_y = target["y"]
            self._target_theta = target["theta"]

        # Trajectory 모드
        self._traj_t: list[float] = []
        self._traj_x: list[float] = []
        self._traj_y: list[float] = []
        self._traj_th: list[float] = []
        if trajectory_points:
            self.set_trajectory(trajectory_points)

        self._warned_no_traj = False  # 궤적 없는 trajectory 모드 경고 1회 제한

    @property
    def has_trajectory(self) -> bool:
        """유효한 궤적(경유점 2개 이상)이 로드되었는지."""
        return len(self._traj_t) >= 2

    def set_target(self, x: float, y: float, theta: float) -> None:
        """단일 목표점을 설정한다."""
        self._target_x = x
        self._target_y = y
        self._target_theta = theta
        self.mode = "target"

    def set_trajectory(self, points: list[dict]) -> None:
        """시간 기반 궤적을 설정한다.

        Args:
            points: [{t, x, y, theta}, ...] 시간 순 정렬
        """
        self._traj_t = [p["t"] for p in points]
        self._traj_x = [p["x"] for p in points]
        self._traj_y = [p["y"] for p in points]
        self._traj_th = [p["theta"] for p in points]
        self.mode = "trajectory"

    def compute(self, state: dict, t: float) -> dict:
        """현재 상태와 시간으로부터 기준 상태를 계산한다.

        Args:
            state: 현재 상태 {x, y, theta, vx, vy, omega}
            t: 현재 시간 [s]

        Returns:
            기준 상태 {x_ref, y_ref, theta_ref, vx_ref, vy_ref, omega_ref}
        """
        if self.mode == "trajectory":
            if self._traj_t:
                return self._compute_trajectory(t)
            if not self._warned_no_traj:
                logger.warning(
                    "[fss_gnc.guidance] trajectory 모드인데 궤적 없음 — target 모드로 동작 중"
                )
                self._warned_no_traj = True
        return self._compute_target(state)

    def _compute_target(self, state: dict) -> dict:
        """Target 모드: 목표점을 직접 기준으로 반환한다."""
        return {
            "x_ref": self._target_x,
            "y_ref": self._target_y,
            "theta_ref": self._target_theta,
            "vx_ref": 0.0,
            "vy_ref": 0.0,
            "omega_ref": 0.0,
        }

    def _compute_trajectory(self, t: float) -> dict:
        """Trajectory 모드: 시간에 따른 보간 기준을 반환한다."""
        # 시간 범위 밖이면 마지막/처음 점 유지
        if t <= self._traj_t[0]:
            return {
                "x_ref": self._traj_x[0], "y_ref": self._traj_y[0],
                "theta_ref": self._traj_th[0],
                "vx_ref": 0.0, "vy_ref": 0.0, "omega_ref": 0.0,
            }
        if t >= self._traj_t[-1]:
            return {
                "x_ref": self._traj_x[-1], "y_ref": self._traj_y[-1],
                "theta_ref": self._traj_th[-1],
                "vx_ref": 0.0, "vy_ref": 0.0, "omega_ref": 0.0,
            }

        # 선형 보간
        idx = np.searchsorted(self._traj_t, t) - 1
        t0, t1 = self._traj_t[idx], self._traj_t[idx + 1]
        alpha = (t - t0) / (t1 - t0)

        x_ref = self._traj_x[idx] + alpha * (self._traj_x[idx + 1] - self._traj_x[idx])
        y_ref = self._traj_y[idx] + alpha * (self._traj_y[idx + 1] - self._traj_y[idx])
        dt_traj = t1 - t0
        vx_ref = (self._traj_x[idx + 1] - self._traj_x[idx]) / dt_traj if dt_traj > 0 else 0.0
        vy_ref = (self._traj_y[idx + 1] - self._traj_y[idx]) / dt_traj if dt_traj > 0 else 0.0

        # 각도 보간 (wrapping 주의)
        dth = self._traj_th[idx + 1] - self._traj_th[idx]
        dth = math.remainder(dth, 2 * math.pi)
        th_ref = self._traj_th[idx] + alpha * dth
        omega_ref = dth / dt_traj if dt_traj > 0 else 0.0

        return {
            "x_ref": x_ref, "y_ref": y_ref, "theta_ref": th_ref,
            "vx_ref": vx_ref, "vy_ref": vy_ref, "omega_ref": omega_ref,
        }
