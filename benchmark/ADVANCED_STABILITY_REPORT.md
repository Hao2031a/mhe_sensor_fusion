# Advanced MHE stability tests

These tests are **offline unit / synthetic benchmarks**, not a Ceres solve against a ROS bag or a real robot. They were run after modifying the C++ estimator.

## Source changes

- Pre-fit event-level NIS soft/hard gating for wheel pair, gyro and accelerometer; no Ceres factor for a hard-rejected sensor; soft gating inflates its **snapshotted sigma**.
- Five accepted MHE solves warmup before gating (configurable). With delayed timestamps, a physically plausible innovation is soft-weighted instead of hard-rejected.
- A rejected wheel event cannot produce a wheel acceleration pair or train common-slip memory. Slip memory updates only after a usable solve when wheel/accel innovations are consistent and sensor health is not degraded.
- Reject nonlinear solutions that are nonfinite, increase objective cost, or change velocity beyond conservative time-scaled physical limits, with Cholesky-to-QR fallback and rollback on failure.
- Fix slightly negative out-of-order timestamps outside merge tolerance: drop instead of inserting a backward transition.
- Propagate covariance of **published integrated pose** (linearization + conservative process floor). Twist covariance still uses Ceres posterior when available. This is an approximate output covariance, not a validated statistical posterior.

## Deterministic / Monte Carlo checks

| Test | Result |
|---|---:|
| Existing upgrade / output stability / 200-trial-per-channel suites | PASS |
| Compiled C++ safety policy assertions | 10 PASS |
| Source integration invariants | 8 PASS |
| Null model, 200,000 normal innovations: soft gate | 0.255% |
| Null model, 200,000 normal innovations: hard gate | 0% observed |
| 12-sigma synthetic spike: hard detection | 99.9965% |
| Synthetic dynamics pre-fit NIS: hard false positives | 0% observed |
| Output pose covariance minimum eigenvalue across simulated 50 s | 0.00016108 (positive) |

The artificial NIS distributions assume correct model variance; real-world vibration, delays and calibration can produce much worse innovations. Do not interpret the 0% hard-gating measurement as a universal guarantee.

## Validation needed on the robot

1. `colcon build --packages-select mhe_sensor_fusion` (not available in the benchmark environment: no ROS 2 / Ceres installation).
2. Run 30-60 s stationary / straight / rotation / figure-eight on real robot; monitor `/mhe/gating_status`, `/mhe/solver_health`, `/mhe/timing`, `/mhe/output_status`, `/odom`.
3. Confirm normal maneuvers do not trigger repeated hard rejects; confirm rollback count normally stays zero. Compare MHE to a separately running EKF and ground truth or loop-closure error.
4. If hard rejects accumulate during legitimate maneuvers, temporarily set `innovation_gate.enabled: false`, keep the recorded rosbag, and calibrate sensor noise/timestamps before tightening thresholds.
