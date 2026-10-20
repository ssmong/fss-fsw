#!/usr/bin/env python3
"""Hydra config 트리 정합성 검증 스크립트.

각 노드의 declare_parameter 목록(EXPECTED)과 Hydra 트리에서 생성되는
파라미터 키가 1:1인지 확인한다. 노드에 파라미터를 추가/삭제하면
이 파일의 EXPECTED도 함께 갱신할 것.

사용법: python scripts/validate_config.py
        (ROS2 불필요 — hydra-core/omegaconf/pyyaml만 있으면 실행 가능)
"""

import math
import sys
import tempfile
from pathlib import Path

# 소싱 없이도 실행 가능하도록 fss_common을 소스 트리에서 직접 로드
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src" / "infra" / "fss_common"))

from fss_common.hydra_bridge import generate_node_param_files, load_hydra_config  # noqa: E402
from fss_common.paths import find_repo_root  # noqa: E402

NODES = [
    "gnc_node", "ekf_node", "valve_driver_node", "manager_node",
    "plant_model_node", "imu_bridge_node", "vicon_bridge_node",
    "arm_driver_node",
]

# 각 노드의 장치·제어 파라미터 (use_sim/use_sim_time/use_thrusters는 런치 주입이라 제외)
EXPECTED = {
    "arm_driver_node": {
        "dry_run", "hardware_confirmed", "port", "baudrate", "return_delay_us", "model_numbers",
        "command_mode", "current_limits_a", "position_profile_velocity", "position_profile_acceleration",
        "control_rate_hz", "command_timeout_s", "mode_timeout_s", "bus_watchdog_s",
        "ids", "directions", "gear_ratios",
        "zero_ticks", "velocity_limits", "lower_limits", "upper_limits",
        "acceleration_limits", "initial_position_rad",
    },
    "gnc_node": {
        "control_rate_hz", "state_age_limit_ms",
        "translational.type", "translational.kp", "translational.ki",
        "translational.kd", "translational.max_force",
        "rotational.type", "rotational.kp", "rotational.ki",
        "rotational.kd", "rotational.max_torque",
        "allocator.thrust_weight", "allocator.min_thrust",
        "allocator.max_thrust_per_nozzle",
        "allocator.thrust_map_file",
        "guidance.mode", "guidance.target_x", "guidance.target_y",
        "guidance.target_theta",
    },
    "ekf_node": {
        "gyro_noise_density", "accel_noise_density", "gyro_bias_random_walk",
        "accel_bias_random_walk", "table_accel_bias_random_walk",
        "vicon_position_noise", "vicon_heading_noise",
        "gate_threshold", "nis_window", "publish_rate_hz",
        "vicon_timeout_ms", "max_prediction_cycles",
    },
    "valve_driver_node": {
        "plc.ip", "plc.port", "plc.timeout_sec", "plc.valve_register_start",
        "plc.valve_count", "plc.main_valve_register", "plc.value_range",
        "plc.deadband_minimum", "plc.control_rate_hz",
        "thrust_map_file", "max_thrust_per_nozzle",
    },
    "manager_node": {
        "health_check_rate_hz", "nav_age_limit_ms",
        "control_rate_min_hz", "auto_safe_hold",
        "nav_pos_std_limit", "nav_heading_std_limit",
    },
    "plant_model_node": {
        "valve_model_mode", "catalog_curve_file", "thrust_map_file",
        "mass", "inertia_zz", "initial_x", "initial_y", "initial_theta",
        "max_thrust_per_valve",
        "pressure_diff_mpa", "signal_current_min_ma", "signal_current_max_ma",
        "signal_to_coil_current_gain", "input_delay_ms", "tau_valve_ms",
        "vicon_rate_hz", "imu_rate_hz", "sim_rate_hz",
        "vicon_position_noise", "vicon_heading_noise",
        "gyro_noise_density", "accel_noise_density",
        "gyro_bias_random_walk", "accel_bias_random_walk",
    },
    "imu_bridge_node": {
        "input_topic", "output_topic", "frame_id", "mounting_quaternion_xyzw",
    },
    "vicon_bridge_node": {
        "bind_ip", "port", "source_ip", "object_name", "poll_rate_hz",
        "vicon_frame_id", "frame_offset_x", "frame_offset_y",
        "frame_rotation_deg",
    },
}

# trajectory 모드에서만 조건부 선언되는 키
OPTIONAL = {
    "gnc_node": {"guidance.traj_t", "guidance.traj_x",
                 "guidance.traj_y", "guidance.traj_theta"},
}

SCENARIOS = [
    ([], "기본 (pid + station_keep)"),
    (["gnc/controller=lqr"], "LQR"),
    (["gnc/guidance=trajectory"], "trajectory 유도"),
    (["experiment=station_keeping"], "실험: station_keeping"),
    (["experiment=trajectory_tracking"], "실험: trajectory_tracking"),
    (["experiment=thrust_characterization"], "실험: thrust_characterization"),
    (["experiment=valve_map_demo"], "실측 밸브 맵 SIL"),
    (
        ["gnc/controller=lqr", "gnc_node.translational.kp=8.0",
         "vehicle.max_thrust_per_nozzle=1.2"],
        "개별 오버라이드 + 보간 전파",
    ),
]


def _flatten(d: dict, prefix: str = "") -> dict:
    out = {}
    for k, v in d.items():
        key = f"{prefix}.{k}" if prefix else k
        if isinstance(v, dict):
            out.update(_flatten(v, key))
        else:
            out[key] = v
    return out


def check(overrides: list[str], label: str) -> bool:
    import yaml

    print(f"\n=== {label} (overrides={overrides}) ===")
    out = Path(tempfile.mkdtemp(prefix="fss_validate_"))
    files = generate_node_param_files(NODES, out, overrides=overrides)
    ok = True
    manager = yaml.safe_load(files["manager_node"].read_text(encoding="utf-8"))
    gnc = yaml.safe_load(files["gnc_node"].read_text(encoding="utf-8"))
    arm = yaml.safe_load(files["arm_driver_node"].read_text(encoding="utf-8"))
    manager_params = manager["manager_node"]["ros__parameters"]
    gnc_params = gnc["gnc_node"]["ros__parameters"]
    rate = manager_params["health_check_rate_hz"]
    if rate != gnc_params["control_rate_hz"]:
        print("  [X] manager와 GNC는 같은 명목 감시 주기를 사용해야 함")
        ok = False
    timeout = arm["arm_driver_node"]["ros__parameters"]["mode_timeout_s"]
    if rate <= 0 or timeout < 2.0 / rate:
        print("  [X] arm mode timeout은 manager 발행 주기의 2배 이상이어야 함")
        ok = False
    for node, path in files.items():
        with open(path, encoding="utf-8") as f:
            content = yaml.safe_load(f)
        got = set(_flatten(content[node]["ros__parameters"]).keys())
        missing = EXPECTED[node] - got
        extra = got - EXPECTED[node] - OPTIONAL.get(node, set())
        if missing:
            print(f"  [X] {node}: 누락 키 {sorted(missing)}")
            ok = False
        if extra:
            print(f"  [X] {node}: 잉여 키 {sorted(extra)} (미선언 — 조용히 무시됨)")
            ok = False
        if not missing and not extra:
            print(f"  [OK] {node}: {len(got)}개 키 정합")
    return ok


def check_hardware_data_files(repo: Path) -> bool:
    """Hydra 외부 장치 설정의 필수 계약과 안전 기본값을 검사한다."""
    import yaml

    print("\n=== Hydra 외부 하드웨어 설정 ===")
    ok = True
    realsense_ok = True

    with open(repo / "config" / "hardware" / "realsense.yaml", encoding="utf-8") as f:
        realsense = yaml.safe_load(f)
    required_realsense = {
        "serial_no",
        "device_type",
        "enable_color",
        "enable_depth",
        "enable_sync",
        "align_depth.enable",
        "pointcloud.enable",
        "enable_gyro",
        "enable_accel",
        "gyro_fps",
        "accel_fps",
        "unite_imu_method",
        "diagnostics_period",
    }
    missing_realsense = required_realsense - set(realsense)
    if missing_realsense:
        print(f"  [X] realsense.yaml: 누락 키 {sorted(missing_realsense)}")
        ok = False
        realsense_ok = False
    if not str(realsense.get("serial_no", "")).startswith("_"):
        print("  [X] realsense.yaml: 숫자 SN은 ROS2 형변환 방지용 '_' 접두사가 필요")
        ok = False
        realsense_ok = False
    if realsense.get("unite_imu_method") != 2:
        print("  [X] realsense.yaml: 보조 IMU 결합은 linear interpolation(2)이어야 함")
        ok = False
        realsense_ok = False
    if realsense.get("accel_fps") not in {100, 200}:
        print("  [X] realsense.yaml: 현재 D435i 세대 accel 프로파일은 100/200Hz만 허용")
        ok = False
        realsense_ok = False
    if realsense.get("pointcloud.enable") is not False:
        print("  [X] realsense.yaml: 기본 화면의 불필요한 부하를 막기 위해 pointcloud는 꺼야 함")
        ok = False
        realsense_ok = False
    if float(realsense.get("diagnostics_period", 0.0)) <= 0.0:
        print("  [X] realsense.yaml: 스트림 상태 확인을 위해 diagnostics_period가 필요")
        ok = False
        realsense_ok = False
    if realsense_ok:
        print("  [OK] realsense.yaml: D435i 장치·영상·보조 IMU 계약 정합")

    with open(repo / "config" / "hardware" / "reaction_wheel.yaml", encoding="utf-8") as f:
        reaction_wheel = yaml.safe_load(f)["reaction_wheel"]
    firmware = reaction_wheel["firmware"]
    connection = reaction_wheel["connection"]
    protocol = reaction_wheel["protocol"]
    electrical = reaction_wheel["electrical"]
    wheel = reaction_wheel["wheel"]
    current_sensor = reaction_wheel["current_sensor"]
    safety = reaction_wheel["safety_evidence"]
    reaction_wheel_ok = True
    if (
        type(reaction_wheel["enabled"]) is not bool
        or reaction_wheel["allow_gnc_commands"] is not False
        or reaction_wheel["integration_state"] not in {"bench_only", "diagnostic"}
    ):
        print("  [X] reaction_wheel.yaml: 독립 bench/diagnostic 드라이버 설정이어야 함")
        ok = False
        reaction_wheel_ok = False
    driver = reaction_wheel["driver"]
    command_timeout = driver["command_timeout_sec"]
    telemetry_timeout = driver["telemetry_timeout_sec"]
    command_rate = connection["command_rate_hz"]
    if (
        not all(math.isfinite(v) and v > 0 for v in (
            command_timeout, telemetry_timeout, command_rate,
        ))
        or not 1.0 / command_rate < command_timeout < connection["rx_timeout_ms"] / 1000.0
    ):
        print("  [X] reaction_wheel.yaml: host timeout 또는 명령 주기 불일치")
        ok = False
        reaction_wheel_ok = False
    if (
        firmware["variant"] != "torque_current_ctrl_simple"
        or firmware["control_law"] != "pure_current_pi"
        or firmware["control_rate_hz"] != 1000.0
        or firmware["pwm_frequency_hz"] != 20000
        or firmware["bemf_feedforward"] is not False
        or firmware["resistance_feedforward"] is not False
        or firmware["coulomb_friction_feedforward"] is not False
        or firmware["speed_observer"] is not False
    ):
        print("  [X] reaction_wheel.yaml: 현재 simple PI 펌웨어 계약 불일치")
        ok = False
        reaction_wheel_ok = False
    if (
        connection["interface"] != "usb_cdc"
        or connection["baudrate"] != 921600
        or connection["telemetry_rate_hz"] != 100.0
        or connection["rx_timeout_ms"] != 500
    ):
        print("  [X] reaction_wheel.yaml: SAMD21 USB-CDC/921600 계약 불일치")
        ok = False
        reaction_wheel_ok = False
    if (
        protocol["command_torque_semantics"] != "wheel"
        or protocol["body_to_wheel_torque_sign"] != -1.0
        or protocol["telemetry_torque_semantics"] != "kt_times_measured_current"
        or protocol["telemetry_has_speed"] is not False
        or protocol["state_codes"] != {"stop": 0, "active": 1, "fault": 2}
    ):
        print("  [X] reaction_wheel.yaml: 명령·상태 telemetry 의미 불일치")
        ok = False
        reaction_wheel_ok = False
    expected_torque = (
        electrical["torque_constant_nm_per_a"] * electrical["continuous_current_limit_a"]
    )
    if not math.isclose(electrical["max_wheel_torque_nm"], expected_torque, rel_tol=1e-6):
        print("  [X] reaction_wheel.yaml: 최대 토크가 KT × current limit와 불일치")
        ok = False
        reaction_wheel_ok = False
    expected_integrator_limit = electrical["bus_voltage_v"] / firmware["current_ki_v_per_as"]
    if not math.isclose(
        firmware["integrator_limit_as"], expected_integrator_limit, rel_tol=1e-6
    ):
        print("  [X] reaction_wheel.yaml: PI integrator clamp가 VBUS/KI와 불일치")
        ok = False
        reaction_wheel_ok = False
    if electrical["hard_current_trip_a"] <= electrical["continuous_current_limit_a"]:
        print("  [X] reaction_wheel.yaml: 과전류 trip이 연속전류 한계보다 커야 함")
        ok = False
        reaction_wheel_ok = False
    if (
        wheel["speed_feedback"] != "none"
        or wheel["has_physical_speed_sensor"] is not False
        or current_sensor["measurement_point"] != "motor_positive_lead"
        or current_sensor["signed_current_expected"] is not True
        or current_sensor["current_sign"] != 1.0
        or current_sensor["shunt_conversion_time_us"] != 588
    ):
        print("  [X] reaction_wheel.yaml: 속도·motor-line 전류 센서 계약 불일치")
        ok = False
        reaction_wheel_ok = False
    required_safety = (
        "stop_active_fault_state_machine_implemented",
        "force_outputs_off_implemented",
        "torque_zero_stop_implemented",
        "immediate_z_stop_implemented",
        "stop_word_implemented",
        "rx_timeout_stop_implemented",
        "overcurrent_fault_implemented",
    )
    if not all(safety[key] is True for key in required_safety):
        print("  [X] reaction_wheel.yaml: 구현된 STOP/FAULT 안전 경로 기록 누락")
        ok = False
        reaction_wheel_ok = False
    if (
        safety["fault_latched_until_stop_command"] is not False
        or safety["nonfinite_current_fault_implemented"] is not False
        or safety["physical_overspeed_protection"] is not False
    ):
        print("  [X] reaction_wheel.yaml: 현재 미구현 안전 항목 기록 불일치")
        ok = False
        reaction_wheel_ok = False
    if reaction_wheel_ok:
        print("  [OK] reaction_wheel.yaml: simple PI 펌웨어 계약·bench_only 가드 정합")

    return ok


def main() -> None:
    """전 시나리오 검증 + 보간 전파 스팟 체크."""
    repo = find_repo_root(Path(__file__).resolve())
    print("repo root:", repo)

    all_ok = all(check(o, label) for o, label in SCENARIOS)
    all_ok = check_hardware_data_files(repo) and all_ok

    # 보간 전파 스팟 체크 (권위 값 1곳 수정 → 소비 노드 반영)
    cfg = load_hydra_config(repo / "config", overrides=["vehicle.max_thrust_per_nozzle=1.5"])
    assert cfg.gnc_node.allocator.max_thrust_per_nozzle == 1.5, "vehicle→gnc 보간 실패"
    assert cfg.valve_driver_node.max_thrust_per_nozzle == 1.5, "vehicle→valve driver 보간 실패"
    assert cfg.plant_model_node.max_thrust_per_valve == 1.5, "vehicle→plant 보간 실패"
    cfg2 = load_hydra_config(repo / "config", overrides=["ekf_node.gyro_noise_density=9e-4"])
    assert cfg2.plant_model_node.gyro_noise_density == 9e-4, "ekf→plant 노이즈 보간 실패"
    cfg3 = load_hydra_config(repo / "config", overrides=[
        "gnc_node.state_age_limit_ms=175.0", "gnc_node.control_rate_hz=40.0",
    ])
    assert cfg3.manager_node.nav_age_limit_ms == 1000.0, "manager의 지속 장애 한계가 GNC와 결합됨"
    assert cfg3.manager_node.health_check_rate_hz == 40.0, "gnc→manager 감시 주기 보간 실패"
    print("\n보간 전파 스팟 체크 통과")

    print("전체 검증 통과" if all_ok else "검증 실패")
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
