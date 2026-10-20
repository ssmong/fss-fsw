# fss_manager

The installed runtime is the C++17 `manager_node` executable. `manager_core`
contains the ROS-independent FSM, health calculations, receipt aging and arming
guards; `manager_ros` adapts the existing ROS topics, service, parameters and
ROS-clock timers. Hydra composes `config/system/manager.yaml` with the full
configuration tree before producing ROS parameter files; loading that fragment
alone does not resolve its cross-node references.
Missing, non-finite or invalid timing/limit parameters stop startup.

GNC's `state_age_limit_ms` (currently 100 ms) zeros control output during a short
navigation gap. Manager's separate `nav_age_limit_ms` (currently 1000 ms) treats
a sustained navigation gap as a fault and latches SAFE_HOLD. A shorter gap can
recover without changing the operating mode. Once manager enters SAFE_HOLD,
it stays latched until an operator requests READY and then ACTIVE after readiness
recovers. Its health timer shares `gnc_node.control_rate_hz` (currently 20 Hz).
Equal nominal periods do not synchronize the two timers or guarantee a response time.
Both timeout settings are provisional, not derived from a plant dynamics guarantee;
tuning requires measured delay, jitter, consecutive dropouts and plant safety
margins. EKF's `vicon_timeout_ms` is a separate sensor-input policy.

`cpp_manager_core` tests all 36 FSM transition pairs, manual E-STOP reset, arming
prerequisites, 1000 ms boundaries, transient recovery, missing/stale receipts, uncertainty faults,
automatic SAFE_HOLD, the disabled automatic-hold option and all invalid wire mode
values. Its fake clock does not sleep. `cpp_manager_ros` instantiates the real
rclcpp adapter and checks topics, service results, immediate mode publication,
parameter failures and health transitions using explicit ROS-time advances.

Readiness is refreshed on the health tick, the
ACTIVE entry guard uses navigation/Vicon/main-valve state, and Vicon health faults
are inferred from EKF uncertainty rather than Vicon receipt age. Expired valve
status clears `actuators_ok`; it does not itself trigger automatic SAFE_HOLD.

Build and test with ROS dependencies available:

```sh
colcon build --packages-up-to fss_manager --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select fss_manager
colcon test-result --verbose
```

The core can also be checked without ROS:

```sh
g++ -std=c++17 -Wall -Wextra -Wpedantic -Werror \
  -Isrc/ops/fss_manager/include src/ops/fss_manager/src/mode_fsm.cpp \
  tests/cpp_manager_test.cpp -o /tmp/cpp_manager_test
/tmp/cpp_manager_test
```
