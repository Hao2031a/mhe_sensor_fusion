# MHE Real-Time Performance Optimization — ROS 2 Kilted

## Scope

No changes to the 9-state dynamics, robust factors, common slip prior, odom/TF
frames, or output smoothing. The runtime costs of the sliding-window optimizer
are targeted without changing estimator semantics deliberately.

### Changes

1. **Ceres linear solver**: `solver.prefer_normal_cholesky` selects
   `DENSE_NORMAL_CHOLESKY` for the primary solve. The original validation
   (finite states, non-increasing cost, bounded endpoint jumps) is retained;
   invalid/nonusable solves retry `DENSE_QR`, then roll back if still invalid.
   Set `prefer_normal_cholesky: false` to restore the previous selection policy.
2. **Covariance SVD throttling**: `covariance.update_every_n: 20` vs 10 before.
   The adaptive schedule defers expensive `ceres::Covariance(DENSE_SVD)` when
   its running latency estimate exceeds the soft 8ms budget. Max age 60
   solves forces occasional updates and publishes covariance inflation while
   estimates age. Failures use a retry cooldown to avoid 100Hz SVD retry loops.
3. **New runtime profile topic**: `/mhe/rt_profile` provides cost breakdown,
   callback inter-arrival time, and cumulative deadline miss counters.
4. **Docker + CI**: Compiled `test_rt_budget`, offline regression unit tests,
   real ROS node profile capture, portable P99/rollback/deadline gate,
   optional strict 100Hz thresholds, and observational comparison to the user's
   previous Docker baseline.

## Important limitations

* Ceres `max_solver_time_in_seconds` and covariance deferral are **soft timing
  budgets**, not preemption. A single covariance/SVD call or QR fallback can
  exceed 10ms. This remains a synchronous ROS timer implementation, not a
  hard real-time background optimizer.
* Slower covariance updates can decrease uncertainty fidelity. Validate NIS,
  empirical error and age on recorded real robot data before use in Nav2.
* Faster Cholesky can be less reliable when linearized Jacobians are ill
  conditioned. Rollback/fallback counts are part of the smoke regression gate.
* Improvement percentages are reported **only** after a new ROS Docker run.
  No local Docker/ROS/Ceres is available in this generation environment.

## Baseline from previous user run

`p50 = 3.689195ms; p95 = 12.80321ms; p99 = 15.276443ms; max = 15.88044ms`
(measured for the previous **total** solve path on the user's Docker host).

## Benchmark in Docker

From the package directory (rebuild image to compile new C++ source):

```bash
docker build -f docker/Dockerfile.ci -t mhe-fusion-ci .
mkdir -p ci-results
docker run --rm --network host \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e CI_ARTIFACT_DIR=/artifacts \
  -v "$(pwd)/ci-results:/artifacts" mhe-fusion-ci
```

Optional strict CI on a dedicated host: add `-e MHE_RT_STRICT=1` to docker run.
This gate can fail if 100Hz targets are not met; do not silently soften them.

Read `ci-results/ros_smoke.json` (realtime_profile), `rt_gate.json`, and
`rt_comparison.json`. The previous `solver_time_ms` remains for backward
compatibility. Use `/mhe/rt_profile` to locate any remaining P99 spikes.

## Offline validations in the delivery environment

* C++ RT scheduling unit tests: 11 checks, PASS.
* Existing C++ stability guard tests: PASS.
* Synthetic and Monte-Carlo benchmark suite: PASS.
* Positive/negative CI regression harness tests: PASS.
* Python syntax and shell script lint: PASS.

**ROS 2/Ceres build and live Docker smoke are not verified in this environment**;
run the Docker commands above on Ubuntu and report the JSON results.
