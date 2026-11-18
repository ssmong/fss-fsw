# Navigation regression tests

The package has one C++ runtime. Python files here only drive compiled executables;
they contain no EKF or alternate navigation implementation.

`ekf_golden.npz` preserves outputs from the verified Python EKF before its removal
on 2026-10-21. The oracle source SHA-256 was
`0a68fa4947e1e68865fb4155c023937ed7fd00a82ade38f765d9e1589db1b796`
(also stored as `oracle_sha256` in the fixture). Its complete original source was
archived before deletion. The current native executable matched all 741 oracle
operations immediately before this fixture was frozen, and an independent replay
through the same oracle verified the saved expectations.

The compressed fixture stores exact input commands and a `741 x 135` float64
expectation matrix. Every row contains acceptance, filter consistency, mean NIS,
11 states, then all 121 covariance entries in row-major order. Parameters are the
MTi-630/MX T10 values in `parity_runner.cpp`. The sequence covers prediction before
initialization, varying sensor intervals, spinning and translational input, all
five bias states, wrapped headings, accepted and rejected observations, rolling
NIS history, zero-length prediction, and reinitialization without clearing NIS.
`test_golden.py` uses the original comparison tolerances: relative `2e-9`, absolute
`2e-11`, with exact acceptance and consistency decisions. The fixture is test data,
not generated during testing or by the implementation under test.

`timed_ekf_golden.npz` separately freezes 1,607 operations from the private Python
`ekf.py` and `timed_ekf.py` used by Self-tossing on 2026-11-18. The SHA-256 of their
concatenated source bytes is
`c8bb0ba1b5d6cc57c9c25e947fcfb7d42f0fe952c61720568d260486143406a3`.
It checks delayed interval splitting/replay, rejected-measurement rollback,
history bounds, bias-preserving recovery, incoherent/frozen observations, and
motion-dependent timestamp uncertainty. Rows contain acceptance, consistency,
mean NIS, recovery/history decisions, update NIS and three innovations, then the
11 states and 121 covariance entries. The same numerical tolerances apply.
Timed initialization and recovery clear NIS, matching that experiment reference;
the raw EKF initialization keeps its existing default for legacy callers.

Coverage retained when removing obsolete Python tests:

| Removed tests | Native replacement |
| --- | --- |
| `tests/test_ekf.py`: covariance-to-standard-deviation, initialization, zero bias | `InitializationAndInvalidInputs`, `CovarianceFailureRemainsVisible` |
| Stationary propagation, growing/shrinking uncertainty, normal observations, NIS | `StationaryPredictionGrowsCovarianceAndViconUpdateShrinksIt`, `GatingWrappedInnovationAndNISWindow`, frozen fixture |
| Midpoint rotation, inertial velocity, gyro correction | `MidpointRotationAndInertialVelocity`, frozen fixture |
| Gyro, body acceleration and table-tilt bias convergence | `GyroBiasConvergesWithinFiveSeconds`, `IsolatedBodyAndTableBiasesMeetOriginalConvergenceLimits`, `BiasConvergenceAndPositiveCovariance` |
| `tests/test_ekf_node.py`: bursts, duplicate/reordered stamps, unavailable/stale/future inputs | `Navigation` C++ tests and the actual executable in `test_ros_node.py` |
| Publication readiness, Vicon duplicates, IMU gaps, stale queued Vicon, outage recovery | `Navigation` C++ tests and `test_ros_node.py` |
| `tests/test_sil_integration.py`: mapped/physical valve convergence at origin and nonzero target | Native GNC, navigation and actuator executables with the Python plant in `tests/test_cxx_runtime.py` |

Run the registered package tests with `colcon test --packages-select fss_navigation`
after building and sourcing the workspace. Closed-loop system tests run separately
from `tests/test_cxx_runtime.py`.
