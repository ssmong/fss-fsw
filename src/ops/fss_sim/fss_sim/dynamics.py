"""3-DOF 평면 강체 동역학 시뮬레이션.

에어베어링 테이블 위 우주선의 운동을 시뮬레이션한다.
마찰 없는 평면 운동을 가정하며, RK4 적분으로 상태를 전파한다.

상태 벡터: [px, py, θ, vx, vy, ω] (테이블 프레임 위치, 본체 프레임 속도)

운동 방정식 (본체 프레임, Newton-Euler):
  m·v̇x = Fx + m·ω·vy   (본체 프레임 속도이므로 transport 항 −ω×v 포함)
  m·v̇y = Fy − m·ω·vx
  Izz·ω̇ = Mz

좌표 변환 (본체→테이블):
  ṗx = vx·cos(θ) − vy·sin(θ)
  ṗy = vx·sin(θ) + vy·cos(θ)
"""

import numpy as np
from numpy.typing import NDArray


class PlanarDynamics:
    """3-DOF 평면 강체 동역학.

    Attributes:
        state: [px, py, θ, vx, vy, ω] 상태 벡터
    """

    # 상태 인덱스
    IPX, IPY, ITH, IVX, IVY, IOM = range(6)
    N = 6

    def __init__(self, mass: float, inertia_zz: float) -> None:
        """동역학 모델 초기화.

        Args:
            mass: 우주선 질량 [kg]
            inertia_zz: Z축 관성 모멘트 [kg·m²]
        """
        self.mass = mass
        self.inertia_zz = inertia_zz
        self.state = np.zeros(self.N)

    def set_state(self, px: float, py: float, theta: float,
                  vx: float = 0.0, vy: float = 0.0, omega: float = 0.0) -> None:
        """상태를 직접 설정한다."""
        self.state = np.array([px, py, theta, vx, vy, omega])

    def _deriv(self, s: NDArray[np.float64], wrench: NDArray[np.float64]) -> NDArray[np.float64]:
        """상태 미분을 계산한다.

        Args:
            s: 상태 [px, py, θ, vx, vy, ω]
            wrench: 본체 프레임 렌치 [Fx, Fy, Mz]

        Returns:
            상태 미분 [ṗx, ṗy, θ̇, v̇x, v̇y, ω̇]
        """
        th = s[self.ITH]
        vx, vy, om = s[self.IVX], s[self.IVY], s[self.IOM]
        fx, fy, mz = wrench[0], wrench[1], wrench[2]

        cos_th = np.cos(th)
        sin_th = np.sin(th)

        ds = np.zeros(self.N)
        # 위치 미분 (본체→테이블)
        ds[self.IPX] = vx * cos_th - vy * sin_th
        ds[self.IPY] = vx * sin_th + vy * cos_th
        ds[self.ITH] = om
        # 속도 미분 (본체 프레임, Newton-Euler — transport 항 −ω×v 포함)
        ds[self.IVX] = fx / self.mass + om * vy
        ds[self.IVY] = fy / self.mass - om * vx
        ds[self.IOM] = mz / self.inertia_zz

        return ds

    def step(self, wrench: NDArray[np.float64], dt: float) -> None:
        """RK4로 한 스텝 전파한다.

        Args:
            wrench: 본체 프레임 렌치 [Fx, Fy, Mz]
            dt: 시간 간격 [s]
        """
        s = self.state
        k1 = self._deriv(s, wrench)
        k2 = self._deriv(s + 0.5 * dt * k1, wrench)
        k3 = self._deriv(s + 0.5 * dt * k2, wrench)
        k4 = self._deriv(s + dt * k3, wrench)

        self.state = s + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        # 각도 정규화
        self.state[self.ITH] = (self.state[self.ITH] + np.pi) % (2.0 * np.pi) - np.pi
