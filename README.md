## Rank-aware Schur prior (experimental Gazebo mode)

Mathematical correctness optimization: information-nullspace preservation with
Jacobi diagonal equilibration and rank-aware square-root construction. See
[`RANK_AWARE_MARGINALIZATION.md`](RANK_AWARE_MARGINALIZATION.md) for derivation,
new solver parameters, rollback, Docker A/B, diagnostics and limits.
Default `config/mhe.yaml` stays on legacy behavior; explicit
`config/mhe_gazebo_low_latency.yaml` enables the experiment.

> **Newest development:** [Timestamp-Aligned Output + Trusted Yaw Correction](TIMESTAMP_YAW_ESTIMATION.md). Stage 1 enabled by default, stage 2 optional/off by default. [Local offline verification](LOCAL_TEST_RESULTS.md).

# mhe_sensor_fusion

ROS 2 Moving Horizon Estimation (MHE) for a differential-drive robot using wheel odometry and IMU.

## Current design

- Time-aligned asynchronous sensor factors using each message's `header.stamp`.
- 100 Hz continuous `/odom` + `odom -> base_footprint` output independent of nonlinear endpoint re-solving.
- Horizon length defined by time (`timing.horizon_duration`) rather than sample count.
- Near-simultaneous IMU and odometry samples are merged into one state.
- Process random-walk noise scales with `sqrt(dt)` for mixed sensor rates.
- ZUPT / stationary detection.
- Gyro and accelerometer bias states.
- Left/right wheel-slip states with observability-aware regularization.
- Schur-complement marginalization.
- Solver rollback on invalid/non-finite solutions.
- Sensor timeout, out-of-order rejection and queue overflow diagnostics.
- Adaptive Q/R with **pre-fit normalized innovation**, EWMA, hysteresis and rate limiting.
- Post-fit residuals are separated from sensor-health statistics so Ceres cannot make a bad sensor look artificially perfect merely by fitting it.

## Pre-fit adaptive R

The important ordering is:

```text
prior MHE state
      |
      v
predict measurement
      |
      +---- measurement ---> pre-fit innovation ---> NIS EWMA ---> hysteresis ---> R scale
      |
      v
warm start + Ceres MHE
      |
      v
post-fit residual (diagnostic only)
```

For a scalar sensor channel, the health statistic uses

```text
innovation = measurement - prior_prediction
NIS ~= innovation^2 / (sigma_measurement^2 + sigma_model^2)
```

`adaptive.prefit_*_model_sigma` represents the prediction uncertainty that is not captured by the measurement sigma. This is a conservative diagonal approximation to `S = H P^- H^T + R`; the current MHE does not maintain a recursive EKF covariance matrix.

Adaptive R is changed only after sustained high NIS and returns to nominal only after sustained low NIS. The scale itself is rate-limited.

## Build

```bash
cd ~/diffbot_ws
rm -rf build/mhe_sensor_fusion install/mhe_sensor_fusion
colcon build --packages-select mhe_sensor_fusion --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Run

```bash
ros2 launch mhe_sensor_fusion mhe.launch.py use_sim_time:=true
```

or

```bash
ros2 run mhe_sensor_fusion mhe_sensor_fusion_node \
  --ros-args --params-file \
  ~/diffbot_ws/src/mhe_sensor_fusion/config/mhe.yaml \
  -p use_sim_time:=true
```

## Main topics

Inputs:

- `/odom/unfiltered`
- `/imu/data`

Outputs:

- `/odom`
- `/tf` (`odom -> base_footprint`)
- `/mhe/confidence`
- `/mhe/innovation`
- `/mhe/postfit_residual`
- `/mhe/bias`
- `/mhe/wheel_slip`
- `/mhe/solve_time_ms`
- `/mhe/marginal_condition`
- `/mhe/sensor_health`
- `/mhe/timing`

### `/mhe/sensor_health`

`Float64MultiArray`:

1. left-wheel **pre-fit NIS EWMA**
2. right-wheel pre-fit NIS EWMA
3. gyro pre-fit NIS EWMA
4. accel pre-fit NIS EWMA
5. left-wheel R scale
6. right-wheel R scale
7. gyro R scale
8. accel R scale
9. left-wheel degraded flag
10. right-wheel degraded flag
11. gyro degraded flag
12. accel degraded flag

The values should no longer decay toward numbers such as `1e-95` merely because Ceres fits the current factor well.

### `/mhe/innovation`

`Float64MultiArray`:

1. left-wheel pre-fit innovation [m/s]
2. right-wheel pre-fit innovation [m/s]
3. gyro pre-fit innovation [rad/s]
4. accel pre-fit innovation [m/s^2]
5. left-wheel R scale
6. right-wheel R scale
7. gyro R scale
8. accel R scale
9. Qv scale
10. Qw scale
11. stationary flag
12. solver-valid flag

### `/mhe/postfit_residual`

`Float64MultiArray`:

1. left-wheel post-fit residual [m/s]
2. right-wheel post-fit residual [m/s]
3. gyro post-fit residual [rad/s]
4. accel post-fit residual [m/s^2]

These are diagnostic only and do not drive adaptive R.

### `/mhe/timing`

`Float64MultiArray`:

1. sensor events processed on the last output tick
2. remaining queue depth
3. total queue-overflow drops
4. total out-of-order drops
5. total horizon resets caused by long sensor gaps
6. age of the newest processed sensor event [s]
7. current horizon duration [s]
8. current number of states

For normal operation the queue depth and all drop counters should remain near zero, and the horizon duration should stay close to `timing.horizon_duration`.

## Quick checks

```bash
ros2 topic echo /mhe/sensor_health --once
ros2 topic echo /mhe/innovation --once
ros2 topic echo /mhe/postfit_residual --once
ros2 topic echo /mhe/timing --once
ros2 topic hz /odom
```

### Adaptive diagnostics

`/mhe/nis_raw` publishes instantaneous pre-fit normalized innovation statistics:

```
[0] left wheel raw NIS
[1] right wheel raw NIS
[2] gyro raw NIS
[3] accelerometer raw NIS
```

`/mhe/sensor_health` publishes the smoothed EWMA NIS values, current R scales and degraded flags. The raw values are useful for transient faults; the EWMA values are what drive the hysteresis gate.

Adaptive R/Q weights are stored per factor/transition when the measurement arrives. This prevents a new sensor anomaly from retroactively changing the weight of every historical factor in the MHE horizon.

## Robustness/scalability upgrade

This revision also targets the four weakest areas found in the previous synthetic benchmark.

### Common-mode slip

Common slip is still fundamentally unobservable during perfectly steady straight motion with only wheel odometry + IMU. The estimator now improves the *observable* portions instead of pretending this limitation does not exist:

- per-state slip observability/sigma snapshots (historical priors are not reweighted by the newest motion state),
- an acceleration-consistency factor during excitation:

```text
wheel_body_accel ~= (1 + common_slip) * (imu_longitudinal_accel - accel_bias)
```

- learned common-slip memory retained during later constant-speed motion,
- exponential memory decay instead of instantly forcing common slip back to zero,
- low-speed gating to avoid dividing/estimating slip near standstill.

`/mhe/wheel_slip` now appends:

```text
[12] learned common-slip memory
[13] current per-state common-slip prior sigma
```

### Accelerometer robustness

The longitudinal accelerometer path now includes:

- configurable sensor-axis projection (`accel.axis`),
- optional orientation-based gravity compensation,
- stationary auto-zero for residual mounting/gravity projection,
- low-pass filtering,
- jerk limiting for single-sample spikes,
- Huber robust loss in the MHE factor.

`/mhe/accel_status`:

```text
[0] projected raw acceleration
[1] low-pass acceleration
[2] learned stationary zero offset
[3] final corrected/jerk-limited acceleration
[4] gravity-compensation-used flag
[5] auto-zero-initialized flag
```

Keep `accel.gravity_compensation: false` unless the IMU orientation is trustworthy. Auto-zero is safe to leave enabled for the normal robot startup-at-rest workflow.

### Posterior covariance

The node now periodically asks Ceres for the posterior covariance block of the newest 9-state MHE state. The block is symmetrized and eigenvalue-clamped before it is mapped into `nav_msgs/Odometry` pose/twist covariance. Unsupported 3D DOFs are explicitly assigned a large variance.

Covariance evaluation is intentionally decimated (`covariance.update_every_n`) and skipped above `covariance.max_states` to avoid disturbing the 100 Hz estimator deadline.

`/mhe/covariance_diag`:

```text
[0] covariance-valid flag
[1] var(x)
[2] var(y)
[3] var(yaw)
[4] var(v)
[5] var(w)
[6] cov(x,y)
[7] cov(v,w)
[8] covariance age in solver updates
[9] cumulative covariance-compute failures
```

### Horizon scalability

- All sensor events drained during one output tick are processed first and then **one** nonlinear solve is performed (`solver.batch_events: true`).
- Prefix marginalization can eliminate multiple expired states in one Schur step, so bursty asynchronous callbacks do not make the horizon grow indefinitely.
- Small horizons use `DENSE_QR`; larger horizons switch to `DENSE_NORMAL_CHOLESKY` and automatically retry with QR if the faster solve is rejected.
- A marginalization failure no longer discards the oldest state; the horizon is kept intact and retried later.

`/mhe/timing` now appends:

```text
[8] states marginalized on the last solve
[9] solver mode: 0=DENSE_QR, 1=DENSE_NORMAL_CHOLESKY, 2=Cholesky->QR fallback
[10] covariance age in solver updates
[11] cumulative nonlinear solve count
```


## Smooth realtime output

The latest build publishes `/mhe/output_status` to compare the raw newest MHE velocity
with the shaped velocity used for `/odom` and `odom -> base_footprint`. The shaping
acts only on V/W and uses small time constants plus acceleration/jerk limits; optimized
pose states are not directly filtered.

## Realtime output stability guard

The MHE trajectory itself remains fully nonlinear and unfiltered. Only the `v/w` target
feeding the continuous `/odom` propagator is guarded:

1. deadband / stationary handling,
2. median-of-3 rejection of one-cycle optimizer spikes,
3. motion-adaptive smoothing time constant,
4. acceleration/deceleration limit,
5. jerk limit,
6. continuous midpoint pose propagation.

This is intentionally different from low-pass filtering `x/y/yaw`; yaw response remains
fast while small solve-to-solve endpoint chatter is attenuated.

Run all offline regression benchmarks with:

```bash
cd <mhe_sensor_fusion_package>
./benchmark/run_benchmarks.sh
```

A successful run ends with `ALL BENCHMARKS PASSED`.

## Advanced stability hardening (2026-10-08)

New ROS parameter groups in `config/mhe.yaml`:

- `innovation_gate.*`: conservative pre-fit event-level normal/soft/hard gating. The wheel measurement is a joint 2-component factor, so either hard outlier rejects that **wheel pair**. Incoming rejected events do not become Ceres measurements. Soft events retain a downweighted factor. Uses approximate (not full covariance-based) NIS.
- `solver.cost_tolerance`, `solver.max_endpoint_*`, `solver.*_jump_margin`: reject unstable nonlinear solutions and roll back. QR fallback remains available for larger horizons.
- `slip.memory_max_prefit_nis`: only update common-slip memory from successful, consistent accel/wheel measurements.
- `/mhe/gating_status` (`std_msgs/msg/Float64MultiArray`) indices `[0..3]` raw NIS left/right/gyro/accel, `[4..7]` event gate state `0=normal,1=soft,2=hard`, `[8..11]` cumulative hard-rejection counts. A wheel-pair rejection is counted by whichever wheel channel(s) failed.
- `/mhe/solver_health` indices `[0]` usable, `[1]` initial cost, `[2]` final cost, `[3]` fractional cost decrease, `[4]` iterations, `[5]` solve ms, `[6]` rollback count, `[7]` solver mode `0=QR,1=Cholesky,2=QR fallback`, `[8..10]` published pose variance `x,y,yaw` (before confidence inflation).
- Published pose covariance now follows the published continuous propagator; no longer copies the internal optimized pose covariance to a different trajectory. Covariance is approximate, not a calibrated posterior.

Run offline unit/synthetic tests after every change:

```bash
./benchmark/run_benchmarks.sh
```

**Caveat:** This suite does not compile the full ROS 2 node or measure real-world accuracy / Ceres runtime. Always rebuild and validate on a ROS 2 Kilted machine. Avoid simultaneous TF broadcasters from EKF and MHE.

## Docker + CI benchmark

See [`ci/README.md`](ci/README.md) for the ROS 2 Kilted Docker image,
colcon/CTest, synthetic stability/innovation benchmark, real ROS node smoke
publisher (100 Hz IMU + 50 Hz wheel odometry), and GitHub Actions workflow.

From package root:

```bash
docker build -f docker/Dockerfile.ci -t mhe-fusion-ci .
mkdir -p ci-results
docker run --rm --network host \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e CI_ARTIFACT_DIR=/artifacts \
  -v "$(pwd)/ci-results:/artifacts" mhe-fusion-ci
```

The test does **not** prove absolute localization accuracy or real-robot
performance; those require a dataset with independent ground truth.

## ROS 2 MHE Real-Time Benchmark

The performance-aware configuration uses Dense Normal Cholesky with automatic
QR fallback and defers periodic Ceres covariance evaluation when insufficient
wall-time headroom remains. Inspect `/mhe/rt_profile` (14 numeric values in
order documented in `ci/README.md`) to distinguish graph-building, Ceres,
covariance, marginalization and callback latency. It is soft real-time only.

With Docker CI, read `ci-results/ros_smoke.json`, `ci-results/rt_gate.json`, and
`ci-results/rt_comparison.json`. Use `MHE_RT_STRICT=1` on a dedicated stable
runner to fail on the 10ms/P99 performance target; do not treat shared-runner
numbers or synthetic tests as proof of hard real-time or hardware accuracy.

## Fast Covariance + Deadline-Aware Scheduling

This package now provides background selected-marginal covariance, a
non-blocking one-slot worker and deadline-aware Jacobian scheduling. See
[FAST_COVARIANCE_NOTES.md](FAST_COVARIANCE_NOTES.md). New diagnostics:
`/mhe/cov_worker_status`. Existing `/mhe/rt_profile[2]` is now *Jacobian
snapshot time* (worker compute time is reported separately). CI adds a
117-column covariance benchmark and optional strict P99/deadline gates.


## Long-duration reliability and covariance consistency

See [LONG_STRESS_CONSISTENCY.md](LONG_STRESS_CONSISTENCY.md) for the 10/30-minute ROS node stress test, fault injection, NEES and approximate NIS diagnostics, strict deadline gate, and Docker/GitHub Actions integration.


## Statistical covariance consistency and calibration

See [STATISTICAL_COVARIANCE_CALIBRATION.md](STATISTICAL_COVARIANCE_CALIBRATION.md) for strict time-aligned NEES, per-axis/phase tests, 30-second-cycle bootstrap, holdout seeds and an opt-in (disabled by default) PSD-preserving covariance calibration.


## Analytic Jacobians for real-time solver

`solver.analytic_factors_enabled: true` uses sparse analytic derivatives for
`ProcessCost` and `WheelPairCost` without changing the Ceres residual, Huber
loss, state space or timing. Set `false` to use the unchanged AutoDiff reference.
See `ANALYTIC_FACTOR_OPTIMIZATION.md`. A 24-horizon Ceres A/B benchmark runs
in Docker CI and writes `ci-results/ceres_analytic_ab.json` (timing advisory).

## Incremental Ceres factor graph and block Schur

This revision can retain Ceres factors between solves (`solver.incremental_graph_enabled`) and marginalize old states with a 9x9 block-tridiagonal Schur chain (`solver.block_schur_enabled`). See [INCREMENTAL_GRAPH_SCHUR.md](INCREMENTAL_GRAPH_SCHUR.md) for the derivation, `/mhe/graph_status` index map, safety details and Docker A/B test. This optimizes graph construction and Schur elimination, **not** Ceres's nonlinear linearization/solve itself. The low-latency angular output remains unchanged. No speedup or accuracy gain on Gazebo is claimed until measured against a controlled rosbag baseline.


### Experimental gyro SO(2) increment

See [GYRO_INCREMENT_SO2_MATH.md](GYRO_INCREMENT_SO2_MATH.md). The experimental
Gazebo YAML enables a bias-aware adjacent-state angular increment factor. It
**replaces** its associated unary gyro factor to prevent double-counting.
Legacy YAML remains unchanged by default. This is *not* full IMU preintegration;
Gazebo ground-truth A/B has not been performed here.
