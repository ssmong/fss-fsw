"""fss_sim 단위 테스트 — 동역학, 밸브 모델, 센서 모델.

실행: pytest tests/test_sim.py -v
"""

import math
from pathlib import Path

import numpy as np
import pytest

from fss_sim.dynamics import PlanarDynamics
from fss_sim.catalog_curve import PVQ30CatalogCurve
from fss_sim.sensor_model import IMUModel, ViconModel
from fss_sim.valve_model import ValveModel


CATALOG_CURVE = PVQ30CatalogCurve.from_yaml(
    Path(__file__).resolve().parents[1]
    / "config"
    / "system"
    / "sim_valve"
    / "pvq33_5g_16_x33_flow_curve.yaml"
)


# ═══════════════════════════════
# dynamics.py
# ═══════════════════════════════


class TestPlanarDynamics:
    """3-DOF 동역학 테스트."""

    def test_zero_force_no_motion(self) -> None:
        """힘 0이면 정지 상태 유지."""
        dyn = PlanarDynamics(mass=10.0, inertia_zz=0.5)
        dyn.set_state(1.0, 2.0, 0.5)
        dyn.step(np.zeros(3), 0.01)
        assert dyn.state[0] == pytest.approx(1.0, abs=1e-10)
        assert dyn.state[1] == pytest.approx(2.0, abs=1e-10)

    def test_constant_force_acceleration(self) -> None:
        """일정한 힘 → 등가속 운동."""
        dyn = PlanarDynamics(mass=10.0, inertia_zz=0.5)
        dyn.set_state(0, 0, 0)  # θ=0, X 전방
        # 1초간 Fx=1N 적용
        for _ in range(1000):
            dyn.step(np.array([1.0, 0.0, 0.0]), 0.001)
        # v = F/m * t = 1/10 * 1 = 0.1 m/s
        assert dyn.state[3] == pytest.approx(0.1, abs=1e-3)
        # x = 0.5 * a * t^2 = 0.5 * 0.1 * 1 = 0.05 m
        assert dyn.state[0] == pytest.approx(0.05, abs=1e-3)

    def test_torque_angular_acceleration(self) -> None:
        """일정 토크 → 각가속도."""
        dyn = PlanarDynamics(mass=10.0, inertia_zz=0.5)
        dyn.set_state(0, 0, 0)
        for _ in range(1000):
            dyn.step(np.array([0.0, 0.0, 0.5]), 0.001)
        # ω = M/I * t = 0.5/0.5 * 1 = 1.0 rad/s
        assert dyn.state[5] == pytest.approx(1.0, abs=1e-3)

    def test_angle_wrapping(self) -> None:
        """각도가 [-π, π) 범위로 정규화되는지 확인."""
        dyn = PlanarDynamics(mass=10.0, inertia_zz=0.5)
        dyn.set_state(0, 0, 3.1)
        # 큰 토크로 π 넘기기
        for _ in range(100):
            dyn.step(np.array([0.0, 0.0, 5.0]), 0.01)
        assert -math.pi <= dyn.state[2] < math.pi


# ═══════════════════════════════
# valve_model.py
# ═══════════════════════════════


class TestValveModel:
    """이상 밸브와 PVQ30 카탈로그 곡선 모델 테스트."""

    @staticmethod
    def _real_model(**overrides: float) -> ValveModel:
        parameters: dict[str, object] = {
            "catalog_curve": CATALOG_CURVE,
            "model_mode": "real",
            "pressure_diff_mpa": 0.7,
            "input_delay_ms": 0.0,
            "tau_valve_ms": 0.0,
        }
        parameters.update(overrides)
        return ValveModel(**parameters)

    def test_ideal_response(self) -> None:
        """이상 모델은 데드밴드나 지연 없이 명령을 추력으로 변환한다."""
        vm = ValveModel(
            max_thrust_per_valve=2.0,
            model_mode="ideal",
            input_delay_ms=100.0,
            tau_valve_ms=100.0,
        )
        thrust = vm.step([0.25] * 8, 0.001)
        assert all(t == pytest.approx(0.5) for t in thrust)
        with pytest.raises(RuntimeError, match="카탈로그 등가 유량"):
            _ = vm.actual_flow_lpm

    def test_zero_command_uses_catalog_zero_flow(self) -> None:
        """X33 4mA 입력은 φ1.6 카탈로그 곡선에서 유량 0이다."""
        vm = self._real_model()
        vm.step([0.0] * 8, 0.01)
        assert np.all(vm.actual_flow_lpm == 0.0)

    def test_x33_scaled_increasing_anchor(self) -> None:
        """X33 14mA→등가 140mA에서 0.7MPa 상승 유량은 85L/min이다."""
        vm = self._real_model()
        duty_for_14ma = (14.0 - 4.0) / (20.0 - 4.0)
        vm.step([duty_for_14ma] * 8, 0.001)
        assert vm.actual_flow_lpm[0] == pytest.approx(85.0)

    def test_x33_scaled_decreasing_anchor(self) -> None:
        """같은 14mA에서 하강 major-loop 유량은 93L/min이다."""
        vm = self._real_model()
        duty_for_14ma = (14.0 - 4.0) / (20.0 - 4.0)
        vm.step([1.0] * 8, 0.001)
        vm.step([duty_for_14ma] * 8, 0.001)
        assert vm.actual_flow_lpm[0] == pytest.approx(93.0)

    def test_pressure_point_anchor(self) -> None:
        """0.35MPa·X33 15mA의 상승 유량은 카탈로그 예시 40L/min이다."""
        vm = self._real_model(pressure_diff_mpa=0.35)
        duty_for_15ma = (15.0 - 4.0) / (20.0 - 4.0)
        vm.step([duty_for_15ma] * 8, 0.001)
        assert vm.actual_flow_lpm[0] == pytest.approx(40.0)

    def test_pressure_interpolation(self) -> None:
        """중간 압력 유량은 인접한 카탈로그 곡선 사이에서 보간된다."""
        current = np.asarray([140.0])
        increasing = np.asarray([True])
        flow_05 = CATALOG_CURVE.flow_lpm(current, 0.5, increasing)[0]
        flow_07 = CATALOG_CURVE.flow_lpm(current, 0.7, increasing)[0]

        vm = self._real_model(pressure_diff_mpa=0.6)
        duty_for_14ma = (14.0 - 4.0) / (20.0 - 4.0)
        vm.step([duty_for_14ma] * 8, 0.001)
        assert vm.actual_flow_lpm[0] == pytest.approx(0.5 * (flow_05 + flow_07))

    def test_pressure_changes_full_scale_thrust(self) -> None:
        """낮은 압력의 최대 유량을 다시 1로 정규화하지 않는다."""
        vm = self._real_model(pressure_diff_mpa=0.5)
        thrust = vm.step([1.0] * 8, 0.001)
        assert vm.full_scale_flow_lpm == pytest.approx(109.0)
        assert vm.actual_flow_lpm[0] == pytest.approx(79.0)
        assert thrust[0] == pytest.approx(79.0 / 109.0)

    def test_pure_input_delay(self) -> None:
        """입력 명령은 설정된 순수 지연이 지난 뒤 적용된다."""
        vm = self._real_model(input_delay_ms=10.0)
        for _ in range(10):
            assert vm.step([1.0] * 8, 0.001)[0] == 0.0
        assert vm.step([1.0] * 8, 0.001)[0] == pytest.approx(1.0)

    def test_catalog_hysteresis_branches(self) -> None:
        """상승·하강 출력 차이는 고정 10%가 아니라 LUT를 따른다."""
        vm = self._real_model()
        duty_for_14ma = (14.0 - 4.0) / (20.0 - 4.0)
        rising = vm.step([duty_for_14ma] * 8, 0.001)[0]
        vm.step([1.0] * 8, 0.001)
        falling = vm.step([duty_for_14ma] * 8, 0.001)[0]
        assert rising == pytest.approx(85.0 / 109.0)
        assert falling == pytest.approx(93.0 / 109.0)
        assert falling - rising == pytest.approx(8.0 / 109.0)

    def test_first_order_response(self) -> None:
        """밸브 시상수 모델은 스텝 명령에 점진적으로 상승한다."""
        vm = self._real_model(tau_valve_ms=20.0)
        thrust_0 = vm.step([1.0] * 8, 0.001)[0]
        for _ in range(100):
            vm.step([1.0] * 8, 0.001)
        thrust_100ms = vm.actual_thrust[0]
        # 100ms 후에는 상당히 수렴했어야 함
        assert thrust_100ms > thrust_0
        assert thrust_100ms > 0.3  # 최대의 60% 이상

    def test_reset(self) -> None:
        """리셋 후 추력 0."""
        vm = self._real_model()
        vm.step([1.0] * 8, 0.01)
        vm.reset()
        assert all(t == 0.0 for t in vm.actual_thrust)
        assert all(flow == 0.0 for flow in vm.actual_flow_lpm)

    def test_invalid_mode_rejected(self) -> None:
        """알 수 없는 모델 모드는 조용히 대체하지 않고 거부한다."""
        with pytest.raises(ValueError, match="지원하지 않는"):
            ValveModel(model_mode="approximate")

    def test_real_mode_requires_catalog_curve(self) -> None:
        """real 모드가 곡선 없이 단순 근사로 fallback하지 않는다."""
        with pytest.raises(ValueError, match="카탈로그 유량 곡선"):
            ValveModel(model_mode="real")

    def test_out_of_catalog_pressure_rejected(self) -> None:
        """압력 범위 밖 값을 끝 곡선으로 clamp하지 않는다."""
        with pytest.raises(ValueError, match="카탈로그 범위"):
            self._real_model(pressure_diff_mpa=0.8)


# ═══════════════════════════════
# sensor_model.py
# ═══════════════════════════════


class TestViconModel:
    """Vicon 센서 노이즈 모델 테스트."""

    def test_noise_statistics(self) -> None:
        """노이즈 통계가 설정값과 일치하는지 확인."""
        np.random.seed(42)
        vicon = ViconModel(position_noise=1e-3, heading_noise=2e-3)
        true_state = np.array([1.0, 2.0, 0.5, 0, 0, 0])

        xs, ys, ths = [], [], []
        for _ in range(5000):
            x, y, th = vicon.measure(true_state)
            xs.append(x - 1.0)
            ys.append(y - 2.0)
            ths.append(th - 0.5)

        assert np.std(xs) == pytest.approx(1e-3, rel=0.2)
        assert np.std(ths) == pytest.approx(2e-3, rel=0.2)


class TestIMUModel:
    """IMU 센서 노이즈 + 바이어스 모델 테스트."""

    def test_bias_random_walk(self) -> None:
        """바이어스가 시간에 따라 드리프트하는지 확인."""
        np.random.seed(42)
        imu = IMUModel(gyro_bias_random_walk=1e-3, accel_bias_random_walk=1e-2)
        state = np.zeros(6)
        wrench = np.zeros(3)
        for _ in range(10000):
            imu.measure(state, wrench, 10.0, 0.001)
        # 10초 후 바이어스가 0에서 드리프트했어야 함
        assert abs(imu.gyro_bias) > 1e-5
