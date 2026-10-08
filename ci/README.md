# MHE Docker + CI benchmark (ROS 2 Kilted)

## What is actually tested

1. `docker build`: compile the *real* ROS 2 Kilted/Ceres node via `colcon build`.
2. `colcon test`, `colcon test-result`, and CTest: compiled C++ stability guard checks, also compiled with normal Release-style flags (checks do not depend on `assert()`).
3. `benchmark/run_benchmarks.sh`: deterministic regressions, Monte-Carlo output smoother, common-slip/accel/covariance synthetic models, and innovation gating Monte-Carlo.
4. `ci/check_regressions.py`: hard thresholds and machine-readable `benchmark_summary.json`.
5. `ci/test_ci_harness.py`: positive/negative regression gate tests and a ROS setup bootstrap regression test for unset `AMENT_TRACE_SETUP_FILES` under `set -u`.
6. `ci/ros_smoke_test.py`: start the actual `mhe_sensor_fusion_node`, publish 100 Hz IMU + 50 Hz odom with valid ROS stamps, inject a wheel spike, assert finite/monotonic odometry and diagnostics, and collect observed Ceres p50/p95/p99 timings.

**Not covered:** Gazebo physics, real wheel-slip ground truth, accuracy vs RTK/motion capture, CPU budget under Nav2, reproducible worst-case execution time, and a true MHE vs EKF accuracy benchmark. The smoke test is explicitly not an estimator accuracy validation. On GitHub shared runners solver timing is a *recorded metric*, not a strict regression gate due to runner variability. For WCET/performance gates use a dedicated self-hosted runner.

## Run locally

From the **package root** (`mhe_sensor_fusion`, containing `CMakeLists.txt`):

```bash
docker build -f docker/Dockerfile.ci -t mhe-fusion-ci .
mkdir -p ci-results
docker run --rm --network host \
  -e ROS_DOMAIN_ID=42 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e CI_ARTIFACT_DIR=/artifacts \
  -v "$(pwd)/ci-results:/artifacts" \
  mhe-fusion-ci
```

Reports: `ci-results/benchmark_summary.json`, `ci-results/ros_smoke.json`,
`ci-results/offline_benchmarks.log`, `ci-results/mhe_node.log`, `ci-results/colcon-log/`.

## GitHub Actions setup

- If **the Git repository root is the `mhe_sensor_fusion` package**, `.github/workflows/mhe-ci.yml` will be discovered automatically.
- If the repository is a **monorepo** (e.g., `diffbot_ws` with package at `src/mhe_sensor_fusion`), copy `mhe_sensor_fusion/.github/workflows/mhe-ci.yml` into **REPOSITORY_ROOT/.github/workflows/mhe-ci.yml** and change the job `PACKAGE_PATH:` to `src/mhe_sensor_fusion`. GitHub Actions only discovers workflows in the repository root `.github/workflows`.
- CI artifacts are uploaded automatically even when tests fail; set branch protection to require `docker-benchmark` before merging.

`ros:kilted-ros-base-noble` is the official ROS image for Ubuntu 24.04.
For strict reproducibility, pin `ROS_IMAGE` by a tested image digest in Dockerfile and archive `dpkg-query -W` output with the result. The unpinned tag can update between runs.

## Benchmark policy

- Every code change: run `benchmark/run_benchmarks.sh`, `ci/check_regressions.py`, CTest and ROS smoke.
- Same-run model baseline thresholds are hard gates.
- **Do not claim** simulation/real accuracy or a wall-clock solver performance improvement based only on synthetic benchmarks.
- Real robot comparisons: record a bag with `/odom/unfiltered`, `/imu/data`, `/odom`, and diagnostic topics, then compare to an independent reference; do not run EKF/MHE simultaneously publishing the same `odom -> base_footprint` TF.

## ROS environment bootstrap (`AMENT_TRACE_SETUP_FILES` error)

`ci/run_ci.sh` temporarily disables Bash `nounset` while sourcing
`/opt/ros/kilted/setup.bash` and `/ws/install/setup.bash`, then reenables it.
This is necessary because the generated ROS setup script may read an unset
`AMENT_TRACE_SETUP_FILES`. A CI harness regression test checks this behavior.

When modifying `ci/run_ci.sh`, rebuild the Docker image before rerunning,
or bind-mount the fixed script into the existing container.

## Real-Time Performance Optimization

This package includes the updated MHE runtime benchmark:

* `/mhe/rt_profile` is `std_msgs/Float64MultiArray` with **14 fields**:
  `[build_ms, ceres_ms, covariance_ms, marginalization_ms, solve_total_ms,
  callback_wall_ms, callback_period_ms, solve_deadline_misses,
  callback_deadline_misses, covariance_deferrals, covariance_forced_refreshes,
  cumulative_solve_count, late_tick_count, solver_rollbacks]`.
* `solver.prefer_normal_cholesky: true` uses Dense Normal Cholesky, with the
  existing Dense QR retry if solution validation fails. This may reduce cost
  but is not guaranteed to do so for every trajectory/CPU.
* `covariance.update_every_n: 20` and `covariance.defer_if_slow: true` defer
  expensive Dense SVD covariance if measured Ceres runtime plus its predicted
  cost exceeds `covariance.rt_budget_ms: 8.0`. It is forced at
  `covariance.max_age_solves: 60` to prevent starvation, and failures have a
  retry cooldown. The published covariance grows conservatively with age.
* These limits are **advisory**. Neither Ceres max_solver_time nor this gate
  can preempt an in-progress SVD or enforce hard real-time 10 ms execution.
* `ci-results/ros_smoke.json`: measured P50/P95/P99 for full solver, Ceres,
  covariance, graph build, marginalization and callbacks, plus missed deadlines.
* `ci-results/rt_gate.json`: portable PASS/FAIL time budget checks.
* `ci-results/rt_comparison.json`: observed relative change vs the previous
  user-provided Docker smoke sample (p99=15.276443ms), not a controlled speedup.

Portable Docker/shared-runner regression thresholds are intentionally looser
(P99 <=25ms, deadline misses <=35% and zero solver rollbacks) because
wall-clock timing varies across machines. For a dedicated performance runner,
set `-e MHE_RT_STRICT=1` when running Docker. This requires P95 <8ms, P99
<=10ms and deadline-miss fractions <=1%. CI **will fail** if these strict
limits are unmet; the baseline already exceeds them. Optionally set
`-e MHE_RT_P99_MAX_MS=12` to override the total solver P99 threshold.

Repeat the Docker benchmark after each optimization and inspect both accuracy
regressions and the real ROS-node timing. Never claim an actual speed-up until
`ros_smoke.json` from the new build is available under comparable conditions.

### Background fast covariance validation

The package now runs `test_fast_covariance` through CTest and executes a
100-trial, 117-column synthetic Jacobian benchmark during Docker CI. It writes
`covariance_worker_benchmark.json` to `$CI_ARTIFACT_DIR`. The ROS smoke test
requires `/mhe/cov_worker_status` updates, at least two valid worker results,
and no worker failures; worker computation latency is recorded separately
from synchronous Ceres `Problem::Evaluate` snapshot duration.

On a controlled self-hosted performance runner, set `MHE_RT_STRICT=1` to
activate P99 solver < 5 ms, callback P99 < 6 ms, callback deadline-miss
fraction < 0.1%, and covariance-age < 120 solves. The default portable gate
cannot be interpreted as a deterministic 100-Hz guarantee.

## Statistical covariance calibration

See [STATISTICAL_COVARIANCE_CALIBRATION.md](../STATISTICAL_COVARIANCE_CALIBRATION.md).
This adds strict timestamp-aligned x/y/yaw NEES, phase diagnostics and
30-second cycle bootstrap; the CI consistency gate intentionally rejects
unproven/overconservative covariance. The new opt-in pose standard-deviation
calibration is **disabled by default** and cannot be automatically activated
by CI. `ci/run_multiseed_stress.sh` provides two+ training seeds and one
independent holdout run; long-duration test results are kept in separate seed
folders instead of overwriting the original `ci-results/long_stress.json`.

## Ceres analytic factor A/B (new)

`ctest` runs the randomized finite-difference Jacobian test and a real
`test_ceres_analytic` factor equivalence test (24 horizons using real Ceres).
The Docker `ci/run_ci.sh` also writes `ci-results/ceres_analytic_ab.json`.
The file records numerical parity **and advisory timing**, not a hardware-neutral
speedup claim. The Gazebo launch defaults to the analytic backend; set
`solver.analytic_factors_enabled: false` for the reference AutoDiff path.
See `ANALYTIC_FACTOR_OPTIMIZATION.md`.

## Ceres analytic factor A/B (new)

`ctest` runs the randomized finite-difference Jacobian test and a real
`test_ceres_analytic` factor equivalence test (24 horizons using real Ceres).
The Docker `ci/run_ci.sh` also writes `ci-results/ceres_analytic_ab.json`.
The file records numerical parity **and advisory timing**, not a hardware-neutral
speedup claim. The Gazebo launch defaults to the analytic backend; set
`solver.analytic_factors_enabled: false` for the reference AutoDiff path.
See `ANALYTIC_FACTOR_OPTIMIZATION.md`.
