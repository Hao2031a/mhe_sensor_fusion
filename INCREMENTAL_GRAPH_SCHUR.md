# Incremental factor graph + 9x9 block-Schur marginalization

## Scope and important limitation

This package retains a **Ceres Problem and its factors across accepted updates**. New measurement states append only their unary and adjacent binary factors. A timestamp-merged IMU/wheel event invalidates and rebuilds the affected newest-state suffix. Successful prefix marginalization deletes stale factor IDs / parameter blocks and adds the new linearized boundary prior in the persistent problem. A Gazebo `/clock` reset destroys the cached graph before freeing state memory. This is an *incremental factor-graph lifecycle*; it is **not** an iSAM2-style incremental nonlinear linear-system factorization. Each Ceres `Solve` still relinearizes and solves the active horizon.

## Marginalization

The state vector at each knot is 9-dimensional. Arrival/slip/wheel/gyro/ZUPT factors are unary; process and longitudinal accelerometer factors couple only adjacent states. Their normal equations are block tridiagonal:

```
[D0 E0  0  ... ]
[E0' D1 E1 ... ]
[0  E1' D2 ... ]
```

For each eliminated block `i`, compute:

`K_i = solve(D_i + λI, E_i)`

`q_i = solve(D_i + λI, b_i)`

`D_{i+1} <- D_{i+1} - E_iᵀ K_i`

`b_{i+1} <- b_{i+1} - E_iᵀ q_i`

The final 9x9 Schur complement becomes the square-root information marginal prior at the retained boundary. This avoids forming a dense `9*(m+1)` matrix on the normal path. Complexity for eliminating `m` states: O(m·9³) time and O(m·9²) additional storage. Sparse factor topology is checked; a non-local factor causes a safe marginalization refusal rather than silently corrupting the prior. Numerical block-LDLT failure triggers a dense-prefix fallback, then safe refusal if both fail.

`Eigen::SelfAdjointEigenSolver` PSD floor and `MarginalPriorCost` assembly are kept as before. The evaluated Ceres residuals and Jacobians use exactly the previous robust-loss evaluation (`EvaluateResidualBlock(..., true, ...)`) and analytic/AutoDiff options.

## Configuration

In the **existing** config YAML:

```yaml
mhe_sensor_fusion:
  ros__parameters:
    solver:
      analytic_factors_enabled: true
      incremental_graph_enabled: true
      block_schur_enabled: true
```

Set **both** new switches false to reproduce legacy graph rebuild + dense Schur (with the same other params). Flags are node startup parameters; restart the MHE node after changing them. The Gazebo YAML retains `output.smoothing.angular_enabled: false`, `use_sim_time: true`, and `transform_time_offset: 0`.

## New runtime diagnostics

`/mhe/graph_status`: `std_msgs/msg/Float64MultiArray`:

| Index | Meaning |
|---|---|
| 0 | Persistent graph enabled (1/0) |
| 1 | Block-Schur enabled (1/0) |
| 2 | Total graph full builds |
| 3 | Cumulative appended/refreshed state factor sets |
| 4 | Successful prefix graph prunes |
| 5 | Graph resets / invalidations |
| 6 | Live Ceres parameter blocks |
| 7 | Live residual blocks |
| 8 | Last graph build/update cost (ms) |
| 9 | Last marginalization cost (ms) |
| 10 | Block-Schur numerical dense fallbacks |

`/mhe/rt_profile` remains unchanged and includes graph-build, Ceres, marginalization, callback timings.

## Verification

1. Offline arithmetic: `OPENBLAS_NUM_THREADS=1 python3 benchmark/benchmark_block_schur.py` compares chain elimination to dense Schur across 144 deterministic synthetic cases. The Python timing measurements are **not** performance predictions for C++/Ceres. The block Python implementation can be *slower* at small horizons because of interpreted-loop overhead.
2. C++ CTest in Docker: `test_block_schur` verifies 864+ block/dense checks; `test_incremental_ceres` compares factor additions / merges / removals versus fresh reconstruction for 48 cycles. These test binaries require Eigen and Ceres and have not been executed in the ChatGPT container.
3. Docker CI builds the full ROS 2 Kilted node and runs an actual ROS2 DDS smoke test on **synthetic sensor stimuli**; `/mhe/graph_status` must be published.
4. `ci/run_graph_ab.sh` runs sequential old/new graph modes of the ROS2 node, generates `/artifacts/graph_ab_comparison.json` and records actual Ceres callback P99 **without asserting a speed-up** across independent wall-time runs.
5. For a rigorous performance/accuracy result, replay the exact same Gazebo rosbag into each variant and compare timestamp-aligned odometry/yaw against Gazebo ground truth, solver rollback and P99; *no Gazebo-ground-truth benchmark has been run here*.

## Safety & compatibility

- `mhe_sensor_fusion` name, topics `/odom`, `/imu/data`, `/odom/unfiltered`, TF `odom -> base_footprint`, Ceres factors, slip priors, and low-latency yaw flags remain unchanged.
- Schur prior is generated only after an accepted solve. Cached factor removal is executed only after successful marginalization; state memory is popped afterward.
- `std::deque` preserves pointers to surviving elements when pushing/popping at ends; the cache additionally checks pointer identity on every solve and can rebuild if it changes.
- Dynamic changes to factor weights, `wheel_separation`, or solver parameters are not supported without explicit cache invalidation; the current node config is read once at startup.
- On `resetHorizon` the cache is destroyed before clearing its state blocks.

## Build / Docker CI

```bash
colcon build --packages-select mhe_sensor_fusion
```

From `mhe_sensor_fusion` package root:

```bash
docker build -f docker/Dockerfile.ci -t mhe-fusion-ci .
mkdir -p ci-results
docker run --rm --network host \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e CI_ARTIFACT_DIR=/artifacts \
  -v "$(pwd)/ci-results:/artifacts" mhe-fusion-ci
```

CI output includes `benchmark_block_schur.json`, `ros_smoke.json`, `graph_ab_full_rebuild.json`, `graph_ab_incremental.json`, `graph_ab_comparison.json`, and regression logs. Use `ros2 topic echo /mhe/graph_status --once` on Gazebo.
