# C++ onboard runtime

This change ports the default onboard ROS2 processes to C++17. It is new implementation work; it does not describe earlier C++ hardware experience. Physical PLC, valves, Vicon, IMU and Jetson scheduling still need hardware revalidation. C++ and selectable CPU affinity/FIFO scheduling alone do not prove bounded latency or hard real-time operation.

## Runtime boundary

| Package | Default executable | Responsibility |
| --- | --- | --- |
| `fss_sensors` | `vicon_bridge_node` | Receive ground-station UDP JSON and publish spacecraft pose |
| `fss_sensors` | `imu_bridge_node` | Transform vendor IMU messages into the body frame |
| `fss_navigation` | `ekf_node` | 11-state IMU/Vicon planar EKF |
| `fss_gnc` | `gnc_node` | Guidance, PID/LQR, allocation and thrust commands |
| `fss_actuators` | `valve_driver_node` | Thrust-map inversion, safety gating, simulated or Modbus TCP PLC outputs |
| `fss_actuators` | `reaction_wheel_driver_node` | Optional USB-CDC reaction-wheel bench driver and dry-run mode |
| `fss_manager` | `manager_node` | Readiness gates, mode FSM and health monitoring |

Node names, topics, messages, services and Hydra parameter names remain the integration contracts. GNC's default executable is now `gnc_node`; `gnc_node.py` remains an explicitly selected research backend. Duplicate Python sensor, navigation, actuator, HAL and manager runtimes have been removed. Calibration math remains in `fss_sensors`; navigation comparison uses a frozen oracle fixture instead of a second EKF implementation.

Python still handles Hydra configuration and launch, experiment tools, calibration, LQR gain computation, dashboards and the SIL plant. The optional Dynamixel arm/vendor SDK adapter retains its existing Python path. The reaction-wheel bench driver is also C++ and keeps its standalone launch path. The Xsens vendor acquisition driver is still started separately with `bash scripts/launch/start_imu.sh`; `sensors.launch.py` starts the body-frame bridge. The optional RealSense launch is unchanged. `use_arm`, `use_thrusters`, `use_gnc`, `use_realsense`, `use_sim_time`, CPU pinning and FIFO priority arguments retain their existing roles. `use_thrusters:=false` also suppresses GNC.

GNC's navigation guard is 100 ms: stale input produces zero thrust and fresh input can resume control. The manager checks at 20 Hz by default and uses a separate 1000 ms navigation-age limit. Exceeding that limit in ACTIVE latches SAFE_HOLD; recovered input alone does not resume ACTIVE. Manual mode transitions and readiness checks are required to rearm.

## Build and launch

From Ubuntu 22.04 with ROS2 Humble sourced:

```bash
bash scripts/install_dependencies.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch fss_bringup sim.launch.py
# Same default C++ runtime, with explicit subsystem options:
ros2 launch fss_bringup fss_full.launch.py use_sim:=true use_arm:=true use_thrusters:=false
python3 scripts/run_experiment.py system.use_sim=true experiment=trajectory_tracking
```

`use_sim:=true` substitutes the Python plant's sensor streams and a C++ simulated PLC backend. It does not connect physical devices. Nodes start in IDLE; launch alone does not arm the system. Follow [mode and main-valve procedures](013_Arm_Execution.md) when operating manually.

Default builds do not require pybind11 or Python development headers for GNC bindings. `FSS_GNC_BUILD_PYTHON_BINDINGS` defaults to `OFF`. To select the retained Python GNC and run Python tests using `_gnc_core`:

```bash
sudo apt-get install pybind11-dev python3-dev
colcon build --symlink-install --cmake-args -DFSS_GNC_BUILD_PYTHON_BINDINGS=ON
source install/setup.bash
ros2 launch fss_bringup gnc.launch.py gnc_backend:=python
ros2 launch fss_bringup sim.launch.py gnc_backend:=python
python3 scripts/run_experiment.py system.use_sim=true system.gnc_backend=python
```

Selecting Python changes only GNC's process; the other default onboard nodes remain C++. Both provided backends support PID/LQR with the same parameter names. LQR uses the checked-in `gnc/controller/lqr_gains.yaml`; `scripts/compute_lqr_gains.py` regenerates it offline using SciPy.

A new Python algorithm is not loaded merely by adding a Hydra YAML group. Implement and explicitly connect it in the retained Python node's `_create_trans_controller` / `_create_rot_controller` path (including parameter declarations and any controller-switch service handling), matching the state/reference and `compute`/`reset` interfaces. Add the Hydra selection and tests, build the optional bindings if retained algorithms still use them, then choose `gnc_backend:=python`. Port a validated algorithm to the C++ node before using it in the default runtime.

## Configuration in source and installed workspaces

`fss_bringup` installs the entire repository `config/` tree under its package share directory, including vehicle geometry, LQR gains, calibration maps and experiment presets. C++ nodes resolve relative configuration files using `FSS_CONFIG_DIR` when set, otherwise `ament_index_cpp` locates `share/fss_bringup/config`. Python helpers use the same environment override and installed share; without an installed package they can fall back to the source tree.

The installed configuration is a build-time copy (or symlink where the build supports it). Rebuild after changing source configuration, or explicitly use the working tree for every process:

```bash
export FSS_CONFIG_DIR="$PWD/config"
FSS_HYDRA_OVERRIDES="gnc/controller=lqr" ros2 launch fss_bringup sim.launch.py
```

`run_experiment.py` composes its source `config/` tree, or the explicit `FSS_CONFIG_DIR` override, and exports that exact directory to the launched processes. This keeps generated ROS parameters and files loaded directly by C++/Python nodes consistent. An invalid explicit directory or file fails visibly rather than choosing another configuration.

## Verification

After building and sourcing the workspace, run the registered C++ tests and the ROS integration checks:

```bash
ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1 colcon test --executor sequential
colcon test-result --verbose
python3 -m pytest -q tests/test_cxx_runtime.py tests/test_launch_options.py \
  tests/test_config_paths.py tests/test_realtime_launch.py \
  tests/test_realtime_launch_graph.py tests/test_run_experiment.py
ROS_DOMAIN_ID=181 ROS_LOCALHOST_ONLY=1 python3 tests/cpp_actuators_ros_smoke.py
```

The C++ integration tests require installed native executables, launch the default chain from outside the source directory with `use_sim:=true`, and verify the running processes are compiled binaries. They check navigation, health, arming, main-valve service, thrust reaching the Python plant and E-stop. Three closed-loop cases preserve the previous SIL criteria: mapped valves reach the origin within 0.05 m after 30 simulated seconds; catalog valves reach the origin within 0.08 m after 45 seconds and a nonzero target within 0.05 m after 30 seconds. Each requires heading error below 0.1 rad. They use localhost ROS domain 179. Sensor bridge inputs are substituted by the plant here; separate native ROS tests exercise actual IMU messages and localhost UDP.

The actuator smoke checks custom calibration, wheel services and limits, ROS-clock pause/resume and output gating. `tests/test_native_thruster_options.py` exercises all four thruster-option/mode combinations with the native driver and Python plant. The arm integration test uses the native manager and valve driver alongside the retained Python dry-run arm. Navigation's 741-operation fixture preserves the former Python oracle's state, covariance, NIS and gating results; native tests retain the bias and covariance convergence checks. Backend and scheduling tests check launch selection and parameter forwarding.

With bindings enabled, `python3 -m pytest -q tests` also runs the retained Python GNC research tests. These checks cover software behavior in their test environment. Record target hardware behavior and timing measurements separately before claiming hardware or deadline validation.

## Verification record — 2026-10-21

The cleanup snapshot was built from empty build/install directories under WSL2 Ubuntu 22.04, ROS2 Humble and GCC 11 on x86-64, using Release and an ordinary copied installation.

- Bindings `OFF`: all 12 packages built. All 35 installed configuration files matched the source. Removed Python runtimes and `_gnc_core` were absent; calibration, plant and research guidance imports resolved to the new installation from outside the checkout.
- Registered native/ROS tests: 79 passed, zero errors, failures or skips (including CTest wrappers). This includes the frozen 741-operation navigation comparison and the separate GNC/manager timeout and recovery checks.
- Four installed native-chain tests passed: default arming/thrust/plant/E-stop smoke and all three original closed-loop criteria. Position errors were 0.0164 m (mapped origin, 30 s), 0.0404 m (catalog origin, 45 s) and 0.0111 m (catalog nonzero target, 30 s); heading errors were below 0.0033 rad.
- Bindings `ON`: the optional module built and imported from the new installation. All seven native executable hashes stayed unchanged. The remaining 241 pytest cases passed, including Python research GNC, native valve-map/option checks, calibration, arm integration and launch tools; the four unchanged native-chain cases above were not repeated.
- The standalone native actuator smoke passed custom calibration, torque-limit rejection, successful wheel command/disable, ROS-clock pause/resume and output gating. During wheel service/command waits its test clock advances so the real monotonic safety timeout remains meaningful. Configuration validation also passed.
- Independent review found no actionable regression, replayed all 741 frozen-oracle operations and checked the clean installed import boundary.

Physical hardware and Jetson timing/deadline validation were not performed.
