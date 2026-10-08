# Odom smooth-output refinement

This build keeps the asynchronous MHE, adaptive pre-fit NIS, common-slip learning,
posterior covariance, batch solve and multi-state marginalization, but isolates the
100 Hz odometry output from high-frequency changes of the newest optimized V/W state.

## Main changes

- Added a realtime V/W output shaper with low time constants and generous physical
  acceleration/jerk limits. Position and yaw are still propagated continuously; they
  are not low-pass filtered directly.
- Stationary/ZUPT and stale-sensor states still snap output twist exactly to zero.
- Reduced adaptive Q scale update speed (`adaptive.q_scale_alpha`).
- Made common-slip memory learning more conservative and the acceleration-slip factor
  weaker, reducing transient coupling from IMU acceleration into V.
- Added `/mhe/output_status`:
  `[raw_v, raw_w, desired_v, desired_w, shaped_v, shaped_w, shaped_a, shaped_alpha,
    common_slip, common_observability, smoothing_enabled, stationary]`.

The goal is to recover the smooth odometry behavior of the earlier stable build while
retaining the estimator improvements introduced later.

# MHE robustness + scalability upgrade

This revision preserves the time-aligned asynchronous MHE, continuous 100 Hz odometry output, ZUPT, adaptive pre-fit NIS, left/right slip states and Schur marginalization, while addressing four benchmark weaknesses.

## 1. Common-mode wheel slip

- Added per-state observability/prior snapshots.
- Added excitation-only wheel-acceleration vs IMU-acceleration common-slip factor.
- Added learned common-slip memory with exponential decay for constant-speed intervals.
- Added low-speed observability gating.
- Added multi-state prefix marginalization so these additional factors remain consistent when multiple states expire in one tick.

Important: absolute common slip during indefinitely constant straight motion is still not fully observable without an independent ground-speed source (LiDAR odometry, visual odometry, GNSS, etc.). This upgrade improves dynamic observability and retention; it does not violate that physical limitation.

## 2. Accelerometer robustness

- Configurable longitudinal IMU axis.
- Optional orientation-based gravity compensation.
- Stationary auto-zero calibration.
- Jerk limiter after LPF.
- Configurable accelerometer Huber delta.
- New `/mhe/accel_status` diagnostics.

## 3. Covariance consistency

- Added posterior newest-state covariance from Ceres rather than only fixed heuristic covariance.
- Symmetry/eigenvalue conditioning before publication.
- Cross-covariances x-y, x-yaw, y-yaw and v-w are published.
- 3D-unobserved DOFs are explicitly marked with large variance.
- Decimated covariance evaluation protects realtime performance.
- New `/mhe/covariance_diag` diagnostics.

## 4. Horizon scalability

- Batch sensor events and solve once per output tick.
- Multi-state prefix Schur marginalization.
- DENSE_QR for small horizon, DENSE_NORMAL_CHOLESKY for larger horizon with QR fallback.
- Failed marginalization no longer pops states.
- `/mhe/timing` extended with marginalized count, solver mode, covariance age and total solve count.

## Stability guard update

This round keeps the MHE optimization model unchanged and improves only the realtime
published velocity targets used by the continuous odometry propagator.

- Added an optional median-of-3 target guard for optimized `v` and `w`.
- Added motion-adaptive smoothing time constants:
  - small solve-to-solve corrections use a larger quiet time constant;
  - genuine maneuvers automatically use the fast time constant.
- Kept the existing acceleration/deceleration and jerk limits.
- Pose `x/y/yaw` is still never low-pass filtered directly.
- Stationary and stale-sensor states still snap exactly to zero.
- Target-history buffers are cleared across horizon/time resets so old samples cannot
  contaminate a restarted estimator.
- `/mhe/output_status` now also publishes guarded targets and effective time constants.

### New `/mhe/output_status` fields

Existing fields `[0..11]` are unchanged. Added:

- `[12]` guarded linear target after median-of-3
- `[13]` guarded angular target after median-of-3
- `[14]` effective linear smoothing tau [s]
- `[15]` effective angular smoothing tau [s]

### Regression benchmark

The package now includes:

- `benchmark/benchmark_upgrade.py`
- `benchmark/benchmark_stability.py`
- `benchmark/benchmark_stability_mc.py`
- `benchmark/run_benchmarks.sh`

The stability Monte-Carlo suite uses 200 trials per channel and rejects the update if
steady-state jitter improvement, step response, or maximum one-cycle output step regress.
