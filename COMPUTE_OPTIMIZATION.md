# MHE factor-graph computation optimization (Gazebo/ROS 2 Kilted)

## Scope

Base: analytic ProcessCost + WheelPairCost, timestamp-aligned publication,
angular smoothing bypass, rate-limited yaw correction (off by default),
solver retry time budgeting, and async covariance worker.

This iteration reduces avoidable **factor graph setup** and **marginalization**
work; it does not change the 9-state MHE residual model, Q/R, robust kernels,
slip constraints, yaw output law, or covariance estimator.

## 1. Persistent Ceres factor graph

`solver.incremental_graph_enabled: true` retains the Ceres Problem across
solves. When a sensor timestamp creates a new knot, only the new unary and
adjacent transition factors are added. A near-synchronous merged IMU/encoder
measurement refreshes the last changed knot's factors. On successful
Schur marginalization, old factor IDs and parameter blocks are removed,
then a new boundary prior is added. A timestamp-reset invalidates the graph
**before** state memory is freed. Pointer identity is checked on every solve.

This is incremental *graph maintenance*, not incremental nonlinear
factorization. Ceres still relinearizes the active horizon on each Solve.

## 2. Block-tridiagonal Schur

`solver.block_schur_enabled: true` accumulates the marginalized prefix into
9x9 diagonal and 9x9 neighboring cross-blocks. It eliminates old knots via

    D_(k+1)' = D_(k+1) - E_k^T (D_k')^-1 E_k
    b_(k+1)' = b_(k+1) - E_k^T (D_k')^-1 b_k

using linear solves, not explicit inversion. The result is a 9x9 boundary
Hessian and RHS, which feed the original SPD-conditioned marginal prior.
The implementation checks numerical validity and can retry the dense
prefix method. Only unary or adjacent pair factors are supported.

The *elimination* phase uses O(m*9^3) arithmetic and O(m*9^2) storage for m
removed knots, versus the dense eliminated-system solve with cubic growth
in m*9. Actual runtime impact depends on Ceres and horizon length.

## 3. Local prefix-factor selection (added in this iteration)

With the cached graph, marginalization looks up only:

* Existing arrival prior at knot 0.
* Unary factors of knots 0..m-1 being eliminated.
* Process/acceleration transition factors ending at knots 1..m.

It **does not include unary factors of the retained boundary knot m**, because
those remain active in the Ceres problem. Including them would double-count
information in the new prior.

The legacy graph-rebuild path retains the old exhaustive scan. If factor types
are added in future, their ownership must be registered in the graph cache,
or local prefix selection must be disabled.

`/mhe/graph_status` retains original fields [0..10] and appends:

* [11] `last_marginal_candidate_count`: residual IDs examined.
* [12] `last_marginal_evaluation_count`: residuals actually Jacobian-evaluated.
* [13] `last_marginal_full_scan`: 1 when using exhaustive Ceres graph scan.

Example **operation count**, not a measured runtime: a 13-state chain with
4 unary factors per knot and 2 transition factors per edge has 77 graph
factors. Removing one old knot requires only 7 candidate IDs (the prior +
4 local + 2 incoming transition), instead of scanning all 77 IDs. This is
90.9% fewer IDs examined *in marginalization*, not a 90.9% faster MHE.

## Config and rollback

New switches default to `false` inside the node for backwards compatibility.
The packaged `config/mhe_gazebo_low_latency.yaml` explicitly enables both;
`config/mhe_user_baseline.yaml` explicitly disables both for A/B comparison.
They are startup-only parameters. Use one MHE node publishing the odom TF.

```yaml
solver:
  analytic_factors_enabled: true
  incremental_graph_enabled: true
  block_schur_enabled: true
```

Leave `output.smoothing.angular_enabled: false`, `use_sim_time: true`, and
`transform_time_offset: 0` for the existing Gazebo yaw test.

## Regressions / what has actually been tested

* C++ (plain g++, no ROS): 1,770 randomized-shape **factor ID selection**
  checks, compiled with `-DNDEBUG`, to catch omissions/duplicates.
* NumPy: 144 synthetic Schur numeric comparisons versus the dense reference.
* Existing Python regression tests / deterministic signal benchmarks.
* Docker image has Ceres/ROS2 CTest, a real ROS2 node smoke test with **synthetic**
  sensor messages and `ci/run_graph_ab.sh` for separately measured graph-build
  P99, Ceres P99, callback P99, rollback and output rate.

**Not yet verified in this environment:** compiling the full ROS2 Kilted/Ceres
node, running the Docker integration suite, or measuring actual Gazebo yaw
RMSE and costmap/SLAM laser alignment. No claim of physical or Gazebo speed-up.

```bash
colcon build --packages-select mhe_sensor_fusion
```

```bash
docker build -f docker/Dockerfile.ci -t mhe-fusion-ci .
mkdir -p ci-results
docker run --rm --network host \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e CI_ARTIFACT_DIR=/artifacts \
  -v "$(pwd)/ci-results:/artifacts" mhe-fusion-ci
```

After the Gazebo launch is restarted, inspect:

```bash
ros2 topic echo /mhe/graph_status --once
ros2 topic echo /mhe/rt_profile --once
ros2 topic echo /mhe/solver_health --once
```

Use same-bag A/B playback and Gazebo ground truth before enabling on the
physical platform. A failure to improve performance is a reason to revert
these two flags without touching previously validated yaw processing.
