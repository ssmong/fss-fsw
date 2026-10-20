"""fss_gnc 단위 테스트 — PID, LQR, QP 할당, Guidance.

실행: pytest tests/test_gnc.py -v
"""

import numpy as np
import pytest

from fss_gnc._gnc_core import (
    LQRTranslational,
    PIDRotational,
    PIDTranslational,
    QPAllocator,
    RotationalRef,
    RotationalState,
    TranslationalRef,
    TranslationalState,
)
from fss_gnc.guidance import GuidanceManager


# B-matrix (3×8, row-major) — thruster_geometry.yaml 기반
B_FLAT = [
    1, 1, -1, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 1, -1, -1,
    -0.05, 0.05, 0.05, -0.05, 0.05, -0.05, -0.05, 0.05,
]


# ═══════════════════════════════
# PID 제어기
# ═══════════════════════════════

class TestPIDTranslational:
    """병진 PID 제어기 테스트."""

    def test_proportional_response(self) -> None:
        """위치 오차에 비례하는 힘 출력."""
        pid = PIDTranslational()
        pid.set_gains(5.0, 0.0, 0.0, 10.0)
        out = pid.compute(
            TranslationalState(1.0, 0.0, 0.0, 0.0),
            TranslationalRef(0.0, 0.0, 0.0, 0.0),
            0.05,
        )
        # Fx = kp * (0 - 1) = -5.0
        assert out.fx == pytest.approx(-5.0, abs=0.01)
        assert out.fy == pytest.approx(0.0, abs=0.01)

    def test_output_saturation(self) -> None:
        """출력이 max_force로 포화."""
        pid = PIDTranslational()
        pid.set_gains(100.0, 0.0, 0.0, 5.0)
        out = pid.compute(
            TranslationalState(1.0, 1.0, 0.0, 0.0),
            TranslationalRef(0.0, 0.0, 0.0, 0.0),
            0.05,
        )
        assert abs(out.fx) <= 5.0
        assert abs(out.fy) <= 5.0

    def test_reset_clears_integral(self) -> None:
        """리셋 후 적분 값이 0."""
        pid = PIDTranslational()
        pid.set_gains(0.0, 1.0, 0.0, 10.0)
        # 적분 누적
        for _ in range(100):
            pid.compute(
                TranslationalState(1.0, 0.0, 0.0, 0.0),
                TranslationalRef(0.0, 0.0, 0.0, 0.0),
                0.05,
            )
        pid.reset()
        out = pid.compute(
            TranslationalState(0.0, 0.0, 0.0, 0.0),
            TranslationalRef(0.0, 0.0, 0.0, 0.0),
            0.05,
        )
        assert out.fx == pytest.approx(0.0, abs=1e-10)


class TestPIDRotational:
    """회전 PID 제어기 테스트."""

    def test_angle_wrapping(self) -> None:
        """큰 각도 오차도 [-π, π]로 정규화."""
        pid = PIDRotational()
        pid.set_gains(1.0, 0.0, 0.0, 5.0)
        # θ = 3.0 → ref = 0.0 → 오차 = -3.0 (wrapping 후 ~3.28)
        out = pid.compute(
            RotationalState(3.0, 0.0),
            RotationalRef(0.0, 0.0),
            0.05,
        )
        # wrapping: remainder(-3.0, 2π) ≈ -3.0 (이미 [-π,π] 범위)
        assert abs(out) <= 5.0  # 포화 범위 내


# ═══════════════════════════════
# LQR 제어기
# ═══════════════════════════════

class TestLQRTranslational:
    """병진 LQR 제어기 테스트."""

    def test_gain_application(self) -> None:
        """K 게인 적용: u = -K @ error."""
        lqr = LQRTranslational()
        # 단위 행렬 게인 (2×4)
        K = [1.0, 0.0, 0.0, 0.0,  # Fx = -1 * ex
             0.0, 1.0, 0.0, 0.0]  # Fy = -1 * ey
        lqr.set_gain(K, 5.0)
        out = lqr.compute(
            TranslationalState(0.5, -0.3, 0.0, 0.0),
            TranslationalRef(0.0, 0.0, 0.0, 0.0),
            0.05,
        )
        # e = state - ref = [0.5, -0.3, 0, 0], u = -K @ e = [-0.5, 0.3]
        assert out.fx == pytest.approx(-0.5, abs=0.01)
        assert out.fy == pytest.approx(0.3, abs=0.01)


# ═══════════════════════════════
# QP 할당기
# ═══════════════════════════════

class TestQPAllocator:
    """QP 기반 추력 할당 테스트."""

    def _make_allocator(self) -> QPAllocator:
        alloc = QPAllocator()
        alloc.set_B_matrix(B_FLAT)
        alloc.thrust_weight = 0.01
        return alloc

    def test_zero_wrench(self) -> None:
        """렌치 0 → 듀티 ~0."""
        alloc = self._make_allocator()
        result = alloc.allocate(0.0, 0.0, 0.0)
        duty = result.duty_list()
        assert all(d < 0.05 for d in duty)

    def test_positive_fx(self) -> None:
        """Fx > 0 → +X 추력기 활성화."""
        alloc = self._make_allocator()
        result = alloc.allocate(2.0, 0.0, 0.0)
        duty = result.duty_list()
        # 추력기 0,1 (+X 방향)이 활성화되어야 함
        assert duty[0] > 0.1 or duty[1] > 0.1

    def test_constraints_respected(self) -> None:
        """모든 듀티가 [0, 1] 범위."""
        alloc = self._make_allocator()
        result = alloc.allocate(5.0, 5.0, 2.0)
        duty = result.duty_list()
        assert all(0.0 <= d <= 1.0 + 1e-6 for d in duty)

    @pytest.mark.parametrize(
        ("wrench", "expected"),
        [
            ((1.0, 0.0, 0.0), (2.0 / 2.01, 0.0, 0.0)),
            ((0.0, -1.0, 0.0), (0.0, -2.0 / 2.01, 0.0)),
            ((0.0, 0.0, 0.1), (0.0, 0.0, 0.05)),
            ((0.0, 0.0, -0.1), (0.0, 0.0, -0.05)),
        ],
    )
    def test_wrench_tracking(self, wrench, expected) -> None:
        """단일 축의 해석적 정규화 최적해를 재현."""
        alloc = self._make_allocator()
        matrix = np.array(B_FLAT).reshape(3, 8)
        duty = np.array(alloc.allocate(*wrench).duty_list())
        assert matrix @ duty == pytest.approx(expected, abs=2e-6)

    @pytest.mark.parametrize("scale", [0.1, 1.0, 10.0])
    @pytest.mark.parametrize("wrench", [(0.7, -0.4, 0.08), (-1.0, -1.1, 0.3), (5.0, 5.0, 2.0)])
    def test_box_optimality(self, scale, wrench) -> None:
        """혼합/포화 명령은 스케일이 달라도 box KKT 조건을 만족."""
        alloc = self._make_allocator()
        # Larger geometry weakens regularization relative to force, requiring more iterations.
        if scale > 1.0:
            alloc.max_iter = 500000
        matrix = scale * np.array(B_FLAT).reshape(3, 8)
        alloc.set_B_matrix(matrix.ravel().tolist())
        requested = scale * np.array(wrench)
        duty = np.array(alloc.allocate(*requested).duty_list())
        assert np.all((duty >= 0.0) & (duty <= 1.0))
        gradient = matrix.T @ (matrix @ duty - requested) + alloc.thrust_weight * duty
        projected_gradient = duty - np.clip(duty - gradient, 0.0, 1.0)
        assert np.max(np.abs(projected_gradient)) < 2e-6

    @pytest.mark.parametrize("iterations", [0, 5000])
    def test_nonzero_lower_bound(self, iterations) -> None:
        """초기화와 반복 결과 모두 설정된 최소 듀티를 준수."""
        alloc = self._make_allocator()
        alloc.min_thrust = 0.2
        alloc.max_thrust = 0.8
        alloc.max_iter = iterations
        duty = np.array(alloc.allocate(0.0, 0.0, 0.0).duty_list())
        assert duty == pytest.approx(np.full(8, 0.2))

    def test_zero_curvature(self) -> None:
        """추력 권한과 비용이 모두 0이어도 유한한 해를 반환."""
        alloc = self._make_allocator()
        alloc.set_B_matrix([0.0] * 24)
        alloc.thrust_weight = 0.0
        assert alloc.allocate(1.0, 2.0, 3.0).duty_list() == [0.0] * 8


# ═══════════════════════════════
# Guidance
# ═══════════════════════════════

class TestGuidanceManager:
    """유도 관리자 테스트."""

    def test_target_mode(self) -> None:
        """target 모드: 목표점을 기준으로 반환."""
        g = GuidanceManager(mode="target", target={"x": 1.0, "y": -0.5, "theta": 0.3})
        ref = g.compute({"x": 0, "y": 0}, 0.0)
        assert ref["x_ref"] == 1.0
        assert ref["y_ref"] == -0.5
        assert ref["theta_ref"] == 0.3

    def test_trajectory_mode_interpolation(self) -> None:
        """trajectory 모드: 시간 보간."""
        points = [
            {"t": 0.0, "x": 0.0, "y": 0.0, "theta": 0.0},
            {"t": 10.0, "x": 1.0, "y": 2.0, "theta": 0.5},
        ]
        g = GuidanceManager(mode="trajectory", trajectory_points=points)
        ref = g.compute({}, 5.0)  # t=5 → 중간점
        assert ref["x_ref"] == pytest.approx(0.5, abs=0.01)
        assert ref["y_ref"] == pytest.approx(1.0, abs=0.01)

    def test_trajectory_hold_at_end(self) -> None:
        """궤적 끝 이후 마지막 점 유지."""
        points = [
            {"t": 0.0, "x": 0.0, "y": 0.0, "theta": 0.0},
            {"t": 5.0, "x": 1.0, "y": 1.0, "theta": 1.0},
        ]
        g = GuidanceManager(mode="trajectory", trajectory_points=points)
        ref = g.compute({}, 100.0)
        assert ref["x_ref"] == 1.0
        assert ref["y_ref"] == 1.0

    def test_set_target_switches_mode(self) -> None:
        """set_target 호출 시 target 모드로 전환."""
        g = GuidanceManager(mode="trajectory")
        g.set_target(2.0, 3.0, 0.1)
        assert g.mode == "target"
        ref = g.compute({}, 0.0)
        assert ref["x_ref"] == 2.0


# ═══════════════════════════════
# LQR 게인 스크립트
# ═══════════════════════════════

class TestLQRGainScript:
    """compute_lqr_gains.py 스크립트 테스트."""

    def test_lqr_gain_computation(self) -> None:
        """LQR 게인 스크립트 실행 + 안정성 확인."""
        from scripts.compute_lqr_gains import build_system, compute_lqr_gain

        A, B = build_system(mass=10.0, izz=0.5)
        Q = np.diag([10.0, 10.0, 5.0, 1.0, 1.0, 0.5])
        R = np.diag([1.0, 1.0, 1.0])
        K = compute_lqr_gain(A, B, Q, R)

        # 폐루프 안정성 확인
        eigvals = np.linalg.eigvals(A - B @ K)
        assert all(ev.real < 0 for ev in eigvals)

    def test_gain_split(self) -> None:
        """게인 분리 (병진 2×4, 회전 1×2) 테스트."""
        from scripts.compute_lqr_gains import build_system, compute_lqr_gain, split_gains

        A, B = build_system(mass=10.0, izz=0.5)
        Q = np.diag([10.0, 10.0, 5.0, 1.0, 1.0, 0.5])
        R = np.diag([1.0, 1.0, 1.0])
        K = compute_lqr_gain(A, B, Q, R)
        K_trans, K_rot = split_gains(K)

        assert len(K_trans) == 8  # 2×4 row-major
        assert len(K_rot) == 2   # 1×2
