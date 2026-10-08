# MHE SO(2) / SE(2) yaw mathematics — Gazebo A/B candidate

This patch advances `mhe_sensor_fusion` **without renaming the package** and
preserves incremental graph, analytic Jacobians, rank-aware Schur prior,
timestamp-aligned odometry, linear-only smoothing, and solver rollback policy.

## What changed

1. **SO(2) yaw residuals**: `wrap(yaw1-yaw0 - average_omega * dt)` instead of
   unrestricted subtraction for `ProcessCost` (AutoDiff and analytic), the
   initial arrival prior, and the marginalized prior. `wrap(d)=atan2(sin d,
   cos d)`; the branch `-pi < d < pi` uses `d` directly for speed. The residual
   is only locally differentiable: at the antipodal +/-pi cut a Ceres
   linearization can be discontinuous. This cut is not relevant to typical
   55 Hz odometry state differences; an exactly 180-degree data association
   ambiguity cannot be solved by wrapping alone.
2. **Exact constant-twist SE(2) integration** for movement between samples:
   with `dtheta=average_omega*dt`,
   `dp=v_avg*dt*sinc(dtheta/2)*[cos(yaw0+dtheta/2), sin(yaw0+dtheta/2)]`.
   Stable `sinc` and derivative Taylor expansions avoid division by zero at
   zero angular velocity. It is exact ONLY under the assumption of constant
   body speed/turn rate over the sample interval. `v_avg` and `omega_avg`
   approximate varying rates as a locally constant twist; this is not exact
   for arbitrary accelerations or slip.
3. **Process analytic Jacobian**: differentiates the `sinc` and midpoint
   heading terms with respect to yaw, v0/v1 and omega0/omega1. Row-major
   Jacobians preserve the same Ceres residual scaling and robust factors.
4. **Continuous output consistency**: state update uses the same arc geometry;
   publisher covariance updates the exact F/G derivatives of the chosen
   integration model. Gyro bias and wheel separation are unchanged.
5. **Backwards compatibility**: ROS node default for both flags is `false`;
   the Gazebo low-latency YAML enables them, while regular `mhe.yaml` and
   `mhe_user_baseline.yaml` keep the old behavior. Angular smoothing remains
   bypassed in the Gazebo YAML.

## Gazebo A/B config

```yaml
solver:
  analytic_factors_enabled: true
  so2_yaw_residual_enabled: true
  exact_se2_motion_enabled: true
```

Switch both flags to `false` to compare with legacy. To isolate the impact,
change only one flag per A/B run. Save matched rosbag ground truth (Gazebo
world pose), `/imu/data`, `/odom/unfiltered`, `/odom`, `/tf`, `/tf_static` and
`/scan` and compare by `header.stamp` in simulation time. Keep only one TF
publisher of `odom -> base_footprint`. Evaluate yaw RMSE and P95 (wrapped
relative yaw), heading lag, turn/corner cross-track drift, solver rollback,
`/mhe/rt_profile` P99 and timestamp gaps.

## Checks actually executable in offline environment

- Standalone C++ random differential Jacobians: 400,802 checks (seed 20261008),
  including +/-2pi invariance, near-zero omega, +/-pi wrap and arc/quadrature.
- Python deterministic independent quadrature: 16 scenarios, 10-100 Hz,
  yaw rates 0-4 rad/s, regression thresholds from report.
- `python3 -m unittest discover -s ci -p 'test_*.py'` and historical synthetic
  regressions via `benchmark/run_benchmarks.sh` and `ci/check_regressions.py`.
- `benchmark/test_ceres_so2_yaw.cpp` compiled/executed ONLY inside ROS/Ceres
  Docker CI or your ROS 2 Kilted workspace; it compares Jet AutoDiff against
  analytic Jacobians and verifies Jet template compilation.
- Docker CI additionally runs real ROS node synthetic-input A/B through
  `ci/run_yaw_so2_ab.sh`. Synthetic-input ROS testing is NOT Gazebo or
  physical-robot ground truth validation.

No ROS, Ceres or Docker executables are installed in the offline editing
container. No ROS2/Gazebo pass is claimed from offline checks.

## Cautions

- New SE(2) integration primarily corrects x/y arc geometry; angular update
  remains `yaw += omega*dt`, so yaw-RMSE improvement is not guaranteed.
- SO(2) wrapping makes equivalent angles indistinguishable. If the estimator
  depends on distinguishing multi-turn unwrapped yaw state, test the impact on
  wheel-slip and gyro-bias estimation before physical deployment.
- Near exactly +/-pi residual discontinuity, use realistic motion bounds and
  appropriate data association; do not blindly increase Ceres iterations.
- `transform_time_offset=0` and Gazebo `use_sim_time=true` are unchanged.
- Do not interpret pure mathematical residual tests as timing/accuracy gains
  in ROS 2 without matched-data A/B.
