"""밸브 맵의 단위·역변환·설정 연결과 지연을 포함한 플랜트 동작."""

import numpy as np
import pytest
from fss_common.hydra_bridge import load_hydra_config
from fss_common.paths import config_dir
from fss_common.valve_thrust_map import ValveThrustMap, load_valve_thrust_map
from fss_sim.valve_model import ValveModel


@pytest.fixture
def curve():
    return load_valve_thrust_map("calibration/valve_thrust_measured.yaml")


def test_measured_linear_interpolation_and_saturation(curve):
    inputs = [0.0, 0.10, 0.20, 0.30, 0.60, 0.80, 1.5, -0.2]
    assert curve.thrust_from_duty(inputs) == pytest.approx(
        [0.0, 0.05, 0.10, 0.15, 0.30, 0.40, 0.50, 0.0]
    )
    assert curve.duty_from_thrust([0.0, -1.0, 2.0, 0.15, 0.30, 0.40, 0.50, 0.50]) == (
        pytest.approx([0.0, 0.0, 1.0, 0.30, 0.60, 0.80, 1.0, 1.0])
    )


def test_roundtrip_across_all_valves(curve):
    rng = np.random.default_rng(1)
    for _ in range(100):
        targets = rng.uniform(0.0, 1.0, 8) * curve.max_thrust_n
        commands = curve.duty_from_thrust(targets)
        assert curve.thrust_from_duty(commands) == pytest.approx(targets, abs=1e-12)
    assert curve.duty_from_thrust([0.0] * 8).tolist() == [0.0] * 8


def test_inverse_handles_flat_sections_without_dividing_by_zero():
    curve = ValveThrustMap([0, 0.2, 0.4, 0.6, 0.8, 1], [[0, 0, 0.5, 0.5, 1, 1]], valve_count=1)
    for target, expected in [(0, 0), (0.25, 0.3), (0.5, 0.4), (0.75, 0.7), (2, 0.8)]:
        assert curve.duty_from_thrust([target])[0] == pytest.approx(expected)


@pytest.mark.parametrize(
    "duty,thrust",
    [
        ([0, 1], [[0, 1]] * 7),
        ([0, 1], [[0, 1, 2]] * 8),
        ([0], [[0]] * 8),
        ([[0, 1]], [[0, 1]] * 8),
        ([0.1, 1], [[0, 1]] * 8),
        ([0, 0.9], [[0, 1]] * 8),
        ([0, 0, 1], [[0, 0.1, 1]] * 8),
        ([0, 1], [[0, float("nan")]] * 8),
        ([0, float("inf")], [[0, 1]] * 8),
        ([0, 1], [[0.1, 1]] * 8),
        ([0, 0.5, 1], [[0, 0.6, 0.5]] * 8),
        ([0, 1], [[0, 0]] * 8),
    ],
)
def test_invalid_maps_fail_at_load(duty, thrust):
    with pytest.raises(ValueError):
        ValveThrustMap(duty, thrust)


@pytest.mark.parametrize("values", [[0] * 7, [0] * 9, [float("nan")] * 8, [float("inf")] * 8])
def test_invalid_commands_are_rejected(curve, values):
    for convert in (curve.thrust_from_duty, curve.duty_from_thrust):
        with pytest.raises(ValueError):
            convert(values)


def test_missing_map_does_not_silently_use_linear_model(tmp_path):
    assert load_valve_thrust_map("") is None
    with pytest.raises(FileNotFoundError):
        load_valve_thrust_map("calibration/missing.yaml")
    path = tmp_path / "bad.yaml"
    path.write_text("duty: [0, 1]\n", encoding="utf-8")
    with pytest.raises(ValueError):
        ValveThrustMap.from_yaml(path)
    with pytest.raises(ValueError):
        ValveModel(model_mode="mapped")


def test_demo_uses_one_map_for_all_nodes():
    cfg = load_hydra_config(config_dir(), overrides=["experiment=valve_map_demo"])
    assert cfg.experiment.name == "valve_map_demo"
    assert cfg.system.use_sim
    assert cfg.plant_model_node.valve_model_mode == "mapped"
    assert cfg.valve_thrust_map_file == cfg.gnc_node.allocator.thrust_map_file
    assert cfg.valve_thrust_map_file == cfg.valve_driver_node.thrust_map_file
    assert cfg.valve_thrust_map_file == cfg.plant_model_node.thrust_map_file
    assert load_valve_thrust_map(cfg.valve_thrust_map_file) is not None


def test_mapped_plant_uses_absolute_newtons_not_legacy_scale(curve):
    plant = ValveModel(model_mode="mapped", thrust_map=curve, max_thrust_per_valve=123.0)
    assert plant.step([1.0] * 8, 0.01) == pytest.approx(curve.max_thrust_n)
    assert plant.step([0.1] * 8, 0.01) == pytest.approx([0.05] * 8)
    targets = np.linspace(0, 0.5, 8)
    assert plant.step(curve.duty_from_thrust(targets), 0.01) == pytest.approx(targets)


def test_mapped_plant_delay_lag_and_reset(curve):
    plant = ValveModel(model_mode="mapped", thrust_map=curve, input_delay_ms=20, tau_valve_ms=10)
    assert plant.step([1.0] * 8, 0.01).tolist() == [0.0] * 8
    assert plant.step([1.0] * 8, 0.01).tolist() == [0.0] * 8
    assert plant.step([1.0] * 8, 0.01) == pytest.approx(curve.max_thrust_n * (1 - np.exp(-1)))
    plant.reset()
    assert plant.actual_thrust.tolist() == [0.0] * 8
    assert plant.step([0.0] * 8, 0.01).tolist() == [0.0] * 8
