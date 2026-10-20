"""소스가 없는 일반 설치와 명시적 설정 디렉토리를 모두 검사한다."""

import sys
from types import ModuleType

import pytest
import yaml

from fss_common.paths import config_dir, config_file


def installed_share(monkeypatch, share):
    packages = ModuleType("ament_index_python.packages")
    packages.PackageNotFoundError = LookupError
    packages.get_package_share_directory = lambda package: str(share)
    monkeypatch.setitem(sys.modules, "ament_index_python", ModuleType("ament_index_python"))
    monkeypatch.setitem(sys.modules, "ament_index_python.packages", packages)


def test_installed_config_does_not_need_source_tree(tmp_path, monkeypatch):
    monkeypatch.delenv("FSS_CONFIG_DIR", raising=False)
    share = tmp_path / "share/fss_bringup"
    (share / "config/vehicle").mkdir(parents=True)
    config = share / "config/vehicle/mass_properties.yaml"
    config.write_text("vehicle: {}", encoding="utf-8")
    installed_share(monkeypatch, share)
    assert config_dir(start=tmp_path) == share / "config"
    assert config_file("vehicle", "mass_properties.yaml", start=tmp_path) == config


def test_explicit_config_overrides_installed_copy(tmp_path, monkeypatch):
    installed_share(monkeypatch, tmp_path / "missing_share")
    monkeypatch.setenv("FSS_CONFIG_DIR", str(tmp_path))
    assert config_dir() == tmp_path
    with pytest.raises(FileNotFoundError, match="설정 파일 없음"):
        config_file("missing.yaml")


def test_invalid_explicit_config_fails_instead_of_falling_back(tmp_path, monkeypatch):
    monkeypatch.setenv("FSS_CONFIG_DIR", str(tmp_path / "missing"))
    with pytest.raises(FileNotFoundError, match="설정 디렉토리 없음"):
        config_dir()


def test_hydra_generation_uses_installed_config_without_source(tmp_path, monkeypatch):
    from fss_common.hydra_bridge import generate_node_param_files

    monkeypatch.delenv("FSS_CONFIG_DIR", raising=False)
    share = tmp_path / "share/fss_bringup"
    (share / "config").mkdir(parents=True)
    (share / "config/config.yaml").write_text("example_node:\n  gain: 2.0\n", encoding="utf-8")
    installed_share(monkeypatch, share)
    files = generate_node_param_files(["example_node"], tmp_path / "generated")
    assert yaml.safe_load(files["example_node"].read_text(encoding="utf-8")) == {
        "example_node": {"ros__parameters": {"gain": 2.0}},
    }


@pytest.mark.parametrize(
    "experiment",
    ["default", "station_keeping", "trajectory_tracking",
     "thrust_characterization", "valve_map_demo"],
)
@pytest.mark.parametrize(
    "gnc_age_ms,manager_age_ms,rate_hz",
    [(100.0, 1000.0, 20.0), (175.0, 1000.0, 40.0), (100.0, 1500.0, 20.0)],
)
def test_separate_navigation_timeouts_and_shared_rate(
    tmp_path, experiment, gnc_age_ms, manager_age_ms, rate_hz,
):
    """모든 실험에서 GNC·manager 한계는 독립적이고 명목 감시 주기만 공유한다."""
    from pathlib import Path

    from fss_common.hydra_bridge import generate_node_param_files

    overrides = [f"experiment={experiment}"]
    if (gnc_age_ms, rate_hz) != (100.0, 20.0):
        overrides += [
            f"gnc_node.state_age_limit_ms={gnc_age_ms}", f"gnc_node.control_rate_hz={rate_hz}",
        ]
    if manager_age_ms != 1000.0:
        overrides += [f"manager_node.nav_age_limit_ms={manager_age_ms}"]
    files = generate_node_param_files(
        ["gnc_node", "manager_node"], tmp_path, overrides=overrides,
        config_dir=Path(__file__).resolve().parents[1] / "config",
    )
    params = {
        name: yaml.safe_load(path.read_text(encoding="utf-8"))[name]["ros__parameters"]
        for name, path in files.items()
    }
    assert params["gnc_node"]["state_age_limit_ms"] == gnc_age_ms
    assert params["manager_node"]["nav_age_limit_ms"] == manager_age_ms
    assert params["gnc_node"]["control_rate_hz"] == rate_hz
    assert params["manager_node"]["health_check_rate_hz"] == rate_hz
