# MHE Fast Covariance + Deadline-Aware Scheduling

## Scope

This changes only posterior covariance scheduling and related CI instrumentation.
The nine-state MHE dynamics, robust factors, Ceres optimization, slip model,
position propagator and TF rules are unchanged. ROS 2 Kilted + Ceres.

## Architecture

1. After an **accepted** MHE solution, the current Ceres `Problem` creates a
   robustified Jacobian **snapshot** using `Problem::Evaluate`. This step still
   runs synchronously, but is allowed only when enough callback budget remains.
2. A **single-slot, try-lock mailbox** transfers ownership of the Jacobian CRS
   arrays into an independent worker thread. It owns no ROS or Ceres pointers.
3. The worker evaluates `H = J' J`, Cholesky factors it (with very limited
   numerical regularization), and solves only the nine final-state RHS columns:
   `P_last = (H^-1)[last9, last9]`. It does **not** construct a full dense inverse.
4. The publisher **polls** without waiting. The returned 9x9 block is checked
   for PSD, finite values and generation/sequence identity before acceptance.
5. Covariance age uses the **snapshot sequence**, not worker completion tick.
   The last valid covariance is inflated conservatively as it ages. At age >
   `max_stale_solves` it is invalidated; the published twist uses conservative
   fallback uncertainty, not a stale posterior masquerading as a fresh result.
6. A ROS time jump or sensor-gap horizon reset increments the generation, so a
   delayed worker result can never repopulate the new horizon with old covariance.

**Accuracy caveat:** This is a robust Gauss-Newton approximation to posterior
covariance. It neglects off-manifold second derivatives and does not include
full covariance of the nonlinear output smoothing pipeline. It is NOT a
statistically calibrated NEES/ground-truth covariance measurement.

## RT controls (config/mhe.yaml)

- `covariance.async_enabled: true` (default). Setting false stops posterior
  refresh and switches to conservative fallback, rather than running blocking SVD.
- `covariance.update_every_n: 20` (aim ~5 Hz at 100 solves/s).
- `covariance.max_age_solves: 60`: request refresh as age grows.
- `covariance.max_stale_solves: 120`: invalidate stale posterior and use fallback.
- `covariance.snapshot_budget_ms: 4.5`: suppress Jacobian sampling if the MHE
  solve has already consumed more than this duration.
- `covariance.rt_reserve_ms: 2.5`: leave time for remaining callback work.
- `rt.deadline_ms: 10.0`: diagnostic budget, **not a hard WCET guarantee**.

Both `covariance.rt_budget_ms` and `covariance.rt_reserve_ms` remain in the
configuration for compatibility, but only reserve participates in the new
snapshot gate; large blocking SVD calls have been removed.

## Diagnostics

`/mhe/rt_profile[2]` now measures **synchronous Jacobian snapshot time**, not
background covariance solve time. Do not compare it directly to the previous
Dense SVD latency. `total_solve_ms` and `callback_wall_ms` still measure the
actual synchronous code path, allowing before/after P99 comparison.

New `/mhe/cov_worker_status` (`std_msgs/msg/Float64MultiArray`):

| Index | Meaning |
|---|---|
| 0 | submitted jobs |
| 1 | accepted completed jobs |
| 2 | failed jobs and Jacobian evaluation failures |
| 3 | deferred or dropped submission attempts |
| 4 | discarded obsolete-generation results |
| 5 | age in accepted MHE solves |
| 6 | most recent worker computation time, ms |
| 7 | most recent synchronous Jacobian snapshot time, ms |
| 8 | 1 if posterior valid, else 0 |
| 9 | sequence of most recent accepted covariance |

`/mhe/covariance_diag` remains unchanged for existing monitoring tools.

## Docker CI

From package root:

```bash
docker build -f docker/Dockerfile.ci -t mhe-fusion-ci .
mkdir -p ci-results
docker run --rm --network host \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e CI_ARTIFACT_DIR=/artifacts \
  -v "$(pwd)/ci-results:/artifacts" mhe-fusion-ci
```

Strict mode on a dedicated fixed-load benchmark machine:

```bash
docker run --rm --network host \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e MHE_RT_STRICT=1 \
  -e CI_ARTIFACT_DIR=/artifacts \
  -v "$(pwd)/ci-results:/artifacts" mhe-fusion-ci
```

Portable CI checks build/functional results, worker liveness, 12 baseline
stability guards, 15 scheduler guards and worker covariance tests. Strict CI
also demands solver P99 < 5 ms, callback P99 < 6 ms, callback deadline-miss
fraction < 0.1%, covariance age < 120 solves and 0 worker failures.
These are **targets**, not measured results for this release. Shared GitHub
runners should generally use portable mode; strict timing mode belongs on a
controlled self-hosted runner with CPU policy and repeatable load.

Expected artifacts: `ros_smoke.json` (`realtime_profile` contains worker stats),
`rt_gate.json`, `rt_comparison.json`, `benchmark_summary.json`,
`covariance_worker_benchmark.json`, `mhe_node.log` and colcon logs.

## Validation limits

The included synthetic covariance benchmark uses 117 columns and 100 trials.
It tests the worker mathematical implementation, not Ceres Jacobian runtime
or localization accuracy on real hardware. Run Docker/colcon on the user's ROS
Kilted machine, then replay recorded wheel and IMU bags versus EKF with
independent ground truth before accepting an accuracy improvement claim.
