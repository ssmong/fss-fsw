# Floating Spacecraft Simulator Flight Software

ROS2 flight software for a 3-DOF air-bearing spacecraft simulator propelled by eight cold-gas thrusters. It fuses IMU and Vicon data in an EKF, closes the loop with PID or LQR, and allocates thrust through a constrained QP to PLC-driven proportional valves.

## Highlights
- 11-state EKF: Xsens IMU strapdown with Vicon pose updates, gyro and body/table accel bias states, Joseph-form update, χ² gating.
- C++17 onboard ROS2 runtime: IMU/Vicon bridges, EKF, guidance, PID/LQR, 8-thruster QP allocation, Modbus TCP valve control and manager FSM. The default chain runs directly in C++ without pybind11.
- GNC publishes per-nozzle target thrust in newtons; the valve driver inverts the measured thrust map before sending normalized inputs through the Modbus TCP HAL to the LS PLC. SIL consumes the driver's converted, gated commands.
- Six-mode manager FSM (IDLE → ACTIVE, SAFE_HOLD, E-STOP) and a 3-joint Dynamixel arm driver with position, velocity and current modes.
- Python SIL plant: configured 1 kHz RK4 rigid-body dynamics, digitized SMC PVQ valve curves and sensor noise; Python calibration, gain calculation, dashboards and Hydra experiment presets remain available.
- Optional Python research GNC selected with `gnc_backend:=python`; its C++ algorithm bindings are an explicit build option. New Python controllers require code wiring as well as configuration.
- GNC zeroes thrust after 100 ms without fresh navigation. The manager checks health at 20 Hz and latches SAFE_HOLD after 1 s of navigation silence; restored input requires manual rearming.

This is a new C++ runtime port. Its implementation and SIL checks do not establish prior C++ hardware operation or measured hard real-time deadlines. See [runtime scope and verification](docs/ops/Cxx_Runtime.md).

## Stack
ROS2 Humble · C++17 · Eigen · yaml-cpp · nlohmann/json · Python 3.10 · NumPy/SciPy · Hydra · Dynamixel SDK · Jetson Orin Nano

## Layout
```text
src/       ROS2 packages: GNC, hardware, infra, ops
config/    Hydra config tree and experiment presets
scripts/   Setup, experiment runner, calibration, dashboard
tests/     Native, ROS integration and Python tool/research tests
plc/       PLC ladder project (XG5000)
docs/      Design and operations notes (Korean)
```

## Usage
```bash
bash scripts/install_dependencies.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install && source install/setup.bash
python scripts/validate_config.py
ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1 colcon test --executor sequential
colcon test-result --verbose
python3 -m pytest -q tests/test_cxx_runtime.py              # compiled runtime + Python plant
ros2 launch fss_bringup sim.launch.py                        # SIL
python scripts/run_experiment.py experiment=trajectory_tracking system.use_sim=true
python scripts/run_experiment.py experiment=valve_map_demo
bash scripts/preflight_check.sh && ros2 launch fss_bringup fss_full.launch.py  # hardware
```

The complete `config/` tree is installed under `share/fss_bringup/config`; an ordinary build can run without the source checkout. To run with a working configuration tree, set `FSS_CONFIG_DIR` to its absolute path. `run_experiment.py` passes its Hydra configuration directory to every node automatically. Xsens vendor acquisition still starts separately with `scripts/launch/start_imu.sh`; optional arm and RealSense adapters retain their existing launch paths. The standalone reaction-wheel bench driver is also C++.

For the Python research backend and Python tests that import `_gnc_core`:

```bash
sudo apt-get install pybind11-dev python3-dev
colcon build --symlink-install --cmake-args -DFSS_GNC_BUILD_PYTHON_BINDINGS=ON
source install/setup.bash
ros2 launch fss_bringup sim.launch.py gnc_backend:=python
python scripts/run_experiment.py system.use_sim=true system.gnc_backend=python
python3 -m pytest -q tests
```
