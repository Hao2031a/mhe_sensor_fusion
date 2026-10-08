# Timestamp-Aligned Output + Trusted Yaw Correction (Stages 1–2)

This builds on the previous solver/yaw optimization while keeping the ROS package
name **mhe_sensor_fusion** (no version suffix). Only the stage 1 output alignment
and stage 2 optional yaw feedback are new.

## Stage 1 — Exact simulation-time integration and sensor timestamp eligibility

* The MHE horizon continues to use original sensor `header.stamp` values.
* Before optimization, input events with stamps more than
  `timing.future_tolerance` ahead of the current ROS time are deferred in the
  existing bounded queue. The algorithm scans all queued events to avoid a
  future-dated message blocking otherwise eligible messages.
* `/odom` and the TF are generated from the **same** output pose. Both use the
  actual ROS publication time; to obtain equal stamps keep
  `transform_time_offset: 0.0` (as in the Gazebo YAML).
* For normal, valid data and bounded tick intervals, integrate over the **exact
  ROS clock delta**, not a silently truncated fixed 10-ms guess or 50-ms cap.
  These two operations do not inherently improve estimator yaw accuracy; they
  prevent timestamps and propagation horizons from becoming inconsistent.
* When /clock is paused, do not publish duplicate time stamps. On a backward
  jump, clear the old sensor queue and reset the estimator epoch. On a long
  forward gap (default >50 ms), **hold** the continuous published pose, publish
  zero twist, increase pose uncertainty, and do not claim to know an
  unobserved trajectory. This prioritizes continuity/safety over tracking
  through gaps. Full pose recovery after a skipped interval needs reference
  anchoring or reinitialization; this behavior must be tested under load.
* Events that precede the optimizer horizon and violate the existing
  out-of-order rules are still rejected as before.

Configuration:

```yaml
use_sim_time: true
transform_time_offset: 0.0
timing:
  timestamp_aligned_output_enabled: true
  max_propagation_gap: 0.05
  future_tolerance: 0.002
```

To A/B-test old vs new time handling, switch
`timing.timestamp_aligned_output_enabled` and restart the node.

## Stage 2 — Opt-in trusted optimizer-endpoint yaw correction

The continuous propagated pose prevents jumps from Ceres re-optimization,
but can accumulate yaw bias. Stage 2 uses the accepted Ceres endpoint at its
**measurement timestamp**, extrapolates by `omega * age` to the output time,
and computes a wrapped heading error:

```text
reference_yaw_at_publish = wrap(yaw_endpoint + omega_endpoint * sensor_age)
error = wrap(reference_yaw_at_publish - published_yaw)
step = clamp(gain * error, -max_rate * publish_dt, +max_rate * publish_dt)
```

Correction is applied only when all checks hold:

* Output is neither stationary nor sensor-timeout/clock-gap held.
* An accepted, usable Ceres endpoint exists and its timestamp matches the
  latest reference measurement within `max_reference_skew`.
* Sensor age is nonnegative and `<= max_sensor_age`.
* Confidence and pre-fit gyro NIS pass trust gates.
* Error is bounded by `max_error`; per-tick correction obeys `max_rate * dt`.
* Optional covariance validity, yaw variance and covariance-age gates pass.

When applied, published yaw, `/odom.twist.angular.z`, and TF remain consistent
(the correction derivative is reflected in angular twist); yaw pose covariance
is inflated. Large corrections are rejected rather than jumped into odom TF.
Yaw feedback is **OFF by default** until validated with independent Gazebo truth.

```yaml
output:
  smoothing:
    enabled: true
    linear_enabled: true
    angular_enabled: false
  yaw_feedback:
    enabled: false  # first run stage 1 alone; then opt in for A/B
    gain: 0.15
    max_rate: 0.20
    max_error: 0.20
    max_sensor_age: 0.04
    max_reference_skew: 0.002
    min_confidence: 0.35
    max_gyro_prefit_nis: 16.0
    require_covariance: false
    max_yaw_variance: 0.10
    max_covariance_age_solves: 80
```

## Diagnostics

`/mhe/time_alignment` (`std_msgs/msg/Float64MultiArray`) indexes:

| Index | Value |
|---|---|
| 0 | Delta in simulation ROS time (s) |
| 1 | Effective integrated delta (s) |
| 2 | Latest sensor age (s) |
| 3 | State: 0=advance, 1=first tick, 2=pause, 3=backward jump, 4=forward gap, 5=future sensor, 6=stale sensor |
| 4 | Future sensor events held in bounded queue |
| 5 | Cumulative forward-gap holds |
| 6 | Cumulative backwards /clock resets |
| 7 | Trusted yaw endpoint (1/0) |
| 8 | Age of accepted yaw reference (s) |
| 9 | Last yaw correction step (rad) |
| 10 | Cumulative rejected yaw trust gates |

Existing `/mhe/output_status` has four *appended* values [24:27]; prior
indexes are unchanged. Existing diagnostics are preserved.

## How to validate in Gazebo

1. Stop other publishers of `odom -> base_footprint`.
2. Run the Gazebo package YAML with `use_sim_time: true` and angular bypass.
3. Initially keep `yaw_feedback.enabled=false`, measure independent Gazebo yaw
   vs MHE `/odom` using `ci/evaluate_gazebo_yaw.py` (real Gazebo ground truth
   must be bridged to a separate nav_msgs/Odometry topic).
4. Enable yaw feedback only after reviewing `/mhe/solver_health`,
   `/mhe/rt_profile`, `/mhe/time_alignment` and baseline yaw error.
5. Repeat identical rotate-left, rotate-right, stop-start, square and straight
   trajectories. Compare yaw RMSE, P95, yaw lag, output 100 Hz, TF continuity,
   callback P99, rollback and deadline misses. The correction MUST NOT be
   declared better on Gazebo merely because synthetic results improve.

Build: `colcon build --packages-select mhe_sensor_fusion`

Run: `ros2 launch mhe_sensor_fusion mhe_gazebo_low_latency.launch.py`

Docker CI (from the package root):

```bash
docker build -f docker/Dockerfile.ci -t mhe-fusion-ci .
mkdir -p ci-results
docker run --rm --network host \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e CI_ARTIFACT_DIR=/artifacts \
  -v "$(pwd)/ci-results:/artifacts" mhe-fusion-ci
```

* `benchmark/test_time_aligned_output.cpp`: compiled timestamp eligibility,
  pause, backwards/forward clock jumps, stale and future gate tests.
* `benchmark/test_low_latency_output.cpp`: compiled yaw wrap, trust gates,
  bounded correction, timestamp freshness and covariance guards.
* `benchmark/benchmark_timestamp_yaw.py`: deterministic 30-seed **synthetic
  policy model**, **not** the ROS node or Gazebo (includes an artificial
  0.012-rad/s bias). The synthetic RMSE must not be extrapolated to a physical
  robot or Gazebo ground-truth experiment.
* `ci/ros_smoke_test.py`: validates that the **actual** ROS node publishes the
  new diagnostic schema while old checks continue to pass. Runs inside Docker.

Offline local checks were run with g++/Python, but this environment has no
ROS 2 or Docker executable, so colcon build, ROS integration test and Gazebo
measurement are **not yet verified**.
