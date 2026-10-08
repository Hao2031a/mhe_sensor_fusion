# MHE mathematical optimization: covariance, Schur and SE(2) pose correction

This is **the same `mhe_sensor_fusion` ROS 2 package name**; not a renamed fork.
Built from the prior SO(2) gyro increment + SE(2) exact arc package.

## 1. Mathematics

**Asynchronous covariance worker.** From the Ceres robustified Jacobian `J`, form
`D_ii = 1/sqrt((J^T J)_ii)`, `J_s = J D`, `H_s = J_s^T J_s`. If a scaled LLT succeeds and its estimated reciprocal condition is greater than `covariance.minimum_rcond`, solve only for the final-state selector E:

`P_tail = E^T D (J_s^T J_s)^-1 D E`.

Do not form a full inverse. Use Eigen's `LLT::rcond()` for the *scaled Hessian*; this is NOT a certified condition number. If its estimate is poor, optional `JacobiSVD` is run on `J_s` itself rather than the squared-condition Hessian. To avoid a dangerous Moore–Penrose nullspace zero-variance artifact, this implementation only returns SVD covariance when J_s has full column rank (relative threshold) and singular-value condition meets `maximum_svd_condition`. Otherwise the worker returns `valid=false`, and the node uses a conservative covariance fallback. When scaled-mode covariance eigenvalues exceed the configured maximum variance, the node rejects the posterior rather than clipping an enormous uncertainty down and falsely claiming accuracy. Rank deficiency can be fundamental (wheel slip, bias, map unobservable), and a larger covariance might be appropriate; rejecting a deficient posterior here is safer than suggesting falsely precise values.

The covariance remains a local Gauss–Newton approximation from the robustified residual Jacobian, **not a statistically calibrated ground-truth posterior**. No probability-coverage claims are made.

**Block Schur.** Stack `[E, r]` and use ONE `LDLT::solve()` per eliminated block. Reject entirely uninformative eliminated blocks before damping, so tiny numerical regularization cannot fabricate information.

**Bounded SE(2) correction.** Use the accepted MHE endpoint (at *last sensor stamp*) and propagate it with exact SE(2) constant twist to the current output stamp. Compute `Log(T_out^-1 T_reference)` in the output body frame, scale by `gain`, clip translational and angular increments by `max_linear_rate*dt` and `max_angular_rate*dt`, and compose `T_out := T_out * Exp(delta)`. Disable correction if sensor stale, stationarity, forced clock hold, rejected/untrusted Ceres endpoint, invalid timestamp/reference skew, low confidence or high gyro prefit NIS. Never activate concurrently with old yaw feedback; concurrent enable disables the new SE(2) path.

The published `twist` continues reporting **MHE model velocity** (not pseudo-motion from pose correction); hence during correction it is an estimate of physical twist, not an exact numerical derivative of corrected output pose. Rate caps and diagnostics bound and expose the mismatch. Use very conservative caps during testing, and prefer this OFF until verified against Gazebo ground truth and TF.

## 2. Configurations (all are complete YAML files)

- `config/mhe.yaml`: old behavior unchanged; new features disabled.
- `config/mhe_gazebo_low_latency.yaml`: pre-existing Gazebo reference; new features disabled.
- `config/mhe_gazebo_math_ab.yaml`: experiment only; enables `covariance.jacobi_scaled_enabled` and `output.se2_correction.enabled`. Legacy `output.yaw_feedback.enabled=false` and angular smoothing bypass retained.
- `launch/mhe_gazebo_math_ab.launch.py`: standalone launch; don't launch another MHE at the same time. If your `diffbot_localization` launch already runs the node, update its YAML path instead of running this second launch.

New ROS parameters (node defaults preserve legacy):

```yaml
covariance:
  jacobi_scaled_enabled: false
  svd_fallback_enabled: true
  minimum_rcond: 1.0e-10
  svd_relative_cutoff: 1.0e-11
  maximum_svd_condition: 1.0e10
output:
  se2_correction:
    enabled: false
    gain: 0.12
    max_linear_rate: 0.03
    max_angular_rate: 0.12
    max_error_m: 0.15
    max_error_rad: 0.20
    max_age_sec: 0.04
    max_reference_skew_sec: 0.002
```

## 3. Diagnostics

- `/mhe/cov_worker_status`: existing indices `[0..9]` unchanged. Append `[10]` algorithm ID (0 legacy, 1 Jacobi LLT, 2 SVD, 3 rejected), `[11]` effective rank, `[12]` scaled reciprocal condition estimate.
- `/mhe/output_status`: existing indices `[0..25]` unchanged. Append `[26]` SE2 enabled, `[27]` applied distance in m per tick, `[28]` applied yaw in rad per tick, `[29]` cumulative correction application count.
- `/mhe/rt_profile`: still contains graph/Ceres/marginalization/callback times.

## 4. Tests and gates

**Offline on development container**: 248 covariance checks (rank, scaling, reference inverse, fallback), 9007 SE(2) correction checks, 864 Schur comparisons, 13 original covariance worker checks, 48 Python tests; all pass. Existing complete offline regression benchmarks pass.

**Offline benchmark, 117-column synthetic Jacobian, 120 paired trials, one run**:
legacy covariance p50=0.296ms, p99=1.172ms; scaled LLT p50=0.631ms, p99=1.792ms. The new kernel is slower here. These are not Gazebo timings, and elapsed times vary by CPU load.

**Docker CI**: `ci/run_ci.sh` now includes the two C++ tests, paired covariance kernel benchmark, configuration unit test and `ci/run_se2_covariance_ab.sh` which runs TWO actual ROS nodes with synthetic IMU+wheel inputs, recording JSON for both cases; requires Docker/ROS and has **not been executed in this environment**. Source package includes `docker/Dockerfile.ci`.

**Gazebo A/B validation**: use matching world, motion script, seeds/rosbag and synchronized ground-truth pose. Test stationary, steady turn, start/stop turn, reversal and 360 deg. Compare timestamp-aligned `e_yaw(t)`, yaw RMSE/P95, xy RMSE, pose jumps, publish rate in *simulation time*, solver rollback, covariance SVD/rejection rate, NEES/coverage and callback p99. No claims of improved Gazebo yaw or accuracy until measured. Use `output.se2_correction.enabled=false` to isolate scaled covariance; switch covariance to false to isolate SE2 correction.

## 5. Build / rollback

```bash
colcon build --packages-select mhe_sensor_fusion
```

Restart the node after a build. To roll back both features, use `config/mhe_gazebo_low_latency.yaml`, or set `covariance.jacobi_scaled_enabled=false` and `output.se2_correction.enabled=false`. Existing 9-state MHE, SO(2) residual, SE(2) arc, gyro increment, incremental graph, rank-aware prior and angular-smoothing bypass stay intact.
