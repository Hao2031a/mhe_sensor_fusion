# MHE Long-Duration Stress + Covariance Consistency

This release **extends the existing fast-covariance package**, preserving the
9-state Ceres MHE, output smoother, and asynchronous covariance worker.

## Three levels of validation

1. **Every Docker CI run:** existing CTest + synthetic Monte Carlo + actual
   ROS/Ceres smoke + **25-second fault-cycle stress**. A 30-second repeated
   scenario includes acceleration, turns, ±encoder noise, 20% wheel slip,
   wheel spikes, missing IMU, missing wheel odom, and deliberately late IMU
   timestamps. All scenarios use the real compiled ROS node and real topics.
2. **Long-duration ROS test:** 600 s (10 min) or 1800 s (30 min). Runs exactly
   the same instrumentation, with bounded-memory aggregation. It measures
   output-rate/liveness, monotonic stamps, missed deadlines, Ceres P99,
   covariance-worker age/failures, rollbacks, pose NEES vs independently
   integrated **synthetic** ground truth and approximate pre-fit NIS.
3. **Real robot (separate work):** use wheel+IMU bags alongside independent
   ground truth (motion capture, survey/RTK as applicable). Synthetic-reference
   NEES **does not** prove accuracy on the actual robot. Repeated samples are
   temporally correlated: treat published chi-square thresholds as diagnostics,
   not an independent-sample confidence interval.

### Docker: build and run full CI (including 25s stress)

Run from the **package root**, not `ci-results`:

```bash
docker build -f docker/Dockerfile.ci -t mhe-fusion-ci .
mkdir -p ci-results
docker run --rm --network host \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e CI_ARTIFACT_DIR=/artifacts \
  -v "$(pwd)/ci-results:/artifacts" \
  mhe-fusion-ci
```

### Docker: long-duration test without repeating all other benchmarks

Run the **10-minute** test:

```bash
docker run --rm --network host \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e CI_ARTIFACT_DIR=/artifacts \
  -e MHE_STRESS_SECONDS=600 \
  -v "$(pwd)/ci-results:/artifacts" \
  mhe-fusion-ci /bin/bash /ws/src/mhe_sensor_fusion/ci/run_long_stress.sh
```

Use `MHE_STRESS_SECONDS=1800` for **30 minutes**. Add
`-e MHE_STRESS_CPU_WORKERS=1` to simulate one busy CPU competitor; this is an
optional load test and may worsen wall-clock deadline metrics. Docker's
`--cpus`, `--cpuset-cpus`, or Linux realtime scheduling should only be applied
on a runner where they are intentional and controlled.

If running on a controlled performance runner, add
`-e MHE_STRESS_STRICT=1` to use strict 100-Hz timing thresholds. This is
**separate** from `MHE_RT_STRICT=1`, which controls the earlier short smoke
performance gate.

`MHE_STRESS_CONSISTENCY_GATE=1` is an **optional experimental** fail-gate for
nominal synthetic-ground-truth NEES calibration. It may legitimately fail
while the estimator's covariance model is being tuned. The default keeps
NEES/NIS visible but does **not** claim their statistical calibration.

### Reports

* `ci-results/long_stress.json`: full duration, injection counters, odometry
  frequency, callback and solver percentiles, worker age, pose NEES and
  coverage, pose error summaries, approximate NIS by sensor.
* `ci-results/long_stress_gate.json`: explicit PASS/FAIL for output validity,
  sensor fault execution, worker health, deadline and optional consistency.
* `ci-results/mhe_stress_node.log`: actual ROS node log.
* `ci-results/long_stress.log`: stress runner log with 60-second progress.
* `ci-results/benchmark_consistency_mc.json`: an **independent Gaussian
  calibration unit benchmark** validating that the scoring formula itself
  detects covariance that is overconfident. This is not a node-accuracy test.

View:

```bash
python3 -m json.tool ci-results/long_stress_gate.json
python3 -m json.tool ci-results/long_stress.json
```

### How the score is calculated

Published pose error `e=[x_est-x_ref, y_est-y_ref, wrap(yaw_est-yaw_ref)]` and
its `3x3` pose covariance `P` from `nav_msgs/Odometry.pose.covariance` give:

```
NEES = e.T @ inv(P) @ e
```

The covariance must be finite, symmetric and positive definite before NEES
is scored. The nominal-trajectory 95% chi-squared threshold for 3 dimensions
is `7.814727903`, and coverage is reported. Samples during injected faults,
start-up, and a brief recovery period are **excluded from the nominal
calibration subset**, but remain included in liveness and safety checks.

`/mhe/nis_raw` reports 4 **approximate** pre-fit squared-normalized residuals
(left wheel, right wheel, gyro Z, accel X). Their values are model-dependent,
possibly clipped and correlated, and must not be called exact Gaussian NIS.
The recorded `chi2_1(0.95)=3.841458821` exceedance fraction is diagnostic only.

**Limitations:** ROS executor/timer and CPU/OS scheduling can still jitter;
Docker smoke and stress benchmarks do not prove hard real-time deadlines.
NEES on the actual robot requires independent accurately time-aligned ground
truth. Synthetic encoder/IMU excitation is useful for regression tests but
cannot stand in for vibration, floor slip, mounting errors, or micro-ROS clock
drift on the physical hardware.

### GitHub Actions

* `.github/workflows/mhe-ci.yml` runs standard CI and the 25-second fault cycle
  on every push/PR.
* `.github/workflows/mhe-long-stress.yml` adds `workflow_dispatch` to request
  1, 10, or 30 minutes, saving artifacts even if the test fails.
* If the package is nested under `diffbot_ws/src/`, copy the workflow to the
  Git repo's **root** `.github/workflows` and set `PACKAGE_PATH` correctly.

## Statistical calibration update

The original 100% nominal NEES coverage indicates that covariance requires
further scrutiny, NOT that the estimator's real-world accuracy is validated.
The newer phase-aligned per-axis diagnostics, independent-cycle bootstrap,
optional output covariance scaling and multi-seed holdout protocol are detailed
in [STATISTICAL_COVARIANCE_CALIBRATION.md](STATISTICAL_COVARIANCE_CALIBRATION.md).
