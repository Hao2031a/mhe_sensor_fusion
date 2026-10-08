# Mathematical optimization: rank-aware square-root marginal prior

## What is changed

This change replaces the *final prior construction* after Block-Schur prefix
elimination. The Schur complement, factor residual definitions, and 9-state
model are unchanged. The old implementation clamped every eigenvalue of the
9x9 Schur information matrix to `1e-7`, reconstructed it, and then performed
an LLT. Even directions with nearly zero information acquired a small positive
curvature. Repeating this step across sliding horizons can invent information.

The new feature `solver.rank_aware_prior_enabled` constructs the quadratic
prior **without artificially adding positive curvature to its nullspace**.

Given linearized prior at the retained boundary:

`J(dx) = 1/2 dx' H dx + b' dx + const`,

1. Symmetrize `H` and diagonally equilibrate heterogeneous 9-state units:
   `D_ii = 1/sqrt(max(abs(H_ii), floor))`, `Hn = D H D`.
2. Eigendecompose `Hn = Q diag(lambda) Q'`.
3. Keep only `lambda > relative_cutoff * max_positive_eigenvalue`; reject
   *strongly* negative eigenvalues, rather than turning an indefinite Schur
   complement into a fictitious valid prior.
4. Build rows of `A = diag(sqrt(lambda_kept)) Q_kept' D^{-1}` and offsets
   `c_i = (q_i' D b) / sqrt(lambda_i)`. Null rows and offsets stay zero.
5. For Ceres: residual is `r = A (x - x_ref) + c`, Jacobian is exactly `A`.
   Thus `A' A` reconstructs the retained information matrix and `A' c`
   reconstructs the range-space projection of the gradient. If the discarded
   gradient is too large relative to the original, reject the prior: the
   original singular quadratic would be unbounded along its nullspace.

**Scope and limitations**

- This is local *linearized information consistency*, not guaranteed globally
  consistent covariance or guaranteed better map alignment.
- Tiny numerical nullspace gradient components are projected out;
  `/mhe/graph_status[16]` shows their norm. Large nullspace gradients are
  rejected instead of silently erased, using a tunable safety threshold.
- Numerical rank depends on scale and threshold. Sweep `1e-8 / 1e-9 / 1e-10`
  on the same rosbag, inspect rank, NEES and solver rollback. Avoid assuming
  rank exactly 9 or exactly a target value in real navigation.
- This adds a scaled 9x9 eigendecomposition after Schur, but removes the
  legacy PSD reconstruction and LLT. In one compiled standalone 9x9 A/B,
  the rank-aware kernel was ~4.1% slower (4,915 vs 4,721 ns/prior).
  End-to-end Ceres/ROS runtime **is not established**.
- `mhe.yaml` and `mhe_user_baseline.yaml` retain legacy mode. Only the explicit
  Gazebo low-latency experimental YAML turns it on. Default declaration is off.
- The angular smoothing bypass, timestamp alignment, and QR fallback policies
  are unchanged.

## Parameter and diagnostics

```yaml
solver:
  rank_aware_prior_enabled: true
  prior_eigen_relative_cutoff: 1.0e-9
  prior_max_discarded_gradient_fraction: 1.0e-4
```

Append-only `/mhe/graph_status` fields:

- `[14]`: rank-aware prior enabled flag
- `[15]`: retained effective information rank (0 to 9)
- `[16]`: norm of discarded gradient (original coordinates)
- `[17]`: cumulative successful rank-aware prior updates
- `[18]`: cumulative invalid/indefinite prior rejections

Legacy `[0]..[13]` fields are unchanged. For a fair A/B comparison, ensure
both runs see the **same sensor records and timestamps** and compare not only
marginalization P99 but yaw RMSE, covariance NEES, TF continuity and dropout.

## Verification levels

1. C++ Eigen unit tests verify reconstruction of Hessian and gradient for
   rank-1..rank-9 synthetic matrices; nullspace handling and negative guard.
2. C++ Chain-Schur-to-prior tests verify 672 synthetic Schur cases.
3. NumPy reference checks 360 randomized matrices (informational only).
4. Optional compiled 9x9 kernel A/B records timing without hard-gating.
5. Docker CI builds actual Ceres/ROS 2 node, runs CTest, ROS smoke and A/B
   scripted-sensor comparisons `ci/run_rank_prior_ab.sh`.
6. **Not yet measured:** ROS 2 Kilted in this environment, Gazebo ground truth,
   physical robot, cross-seed NEES under extended trajectories.

## Build

```bash
colcon build --packages-select mhe_sensor_fusion
```

## Docker verification

```bash
docker build -f docker/Dockerfile.ci -t mhe-fusion-ci .
mkdir -p ci-results
docker run --rm --network host -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e CI_ARTIFACT_DIR=/artifacts \
  -v "$(pwd)/ci-results:/artifacts" mhe-fusion-ci
```

Artifacts include `benchmark_rank_aware_prior.json` (synthetic numerical
reference), `rank_prior_legacy.json`, `rank_prior_rank_aware.json`, and
`rank_prior_ab_comparison.json` (actual ROS node with scripted synthetic inputs).
