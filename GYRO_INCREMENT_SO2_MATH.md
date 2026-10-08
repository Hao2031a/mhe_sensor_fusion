# SO(2) bias-aware gyro increment — mathematical A/B extension

**Status:** experimental opt-in only; no Gazebo ground-truth proof. This is a **single-sample interval gyro increment**, *not* multi-keyframe preintegration or iSAM2.

## Mathematical factor

For adjacent retained states `k-1,k`, on an accepted IMU event with `0.5 ms <= dt <= gyro_increment_max_dt`:

```
Delta_yaw_m = dt * (z_g,k - 0.5 * (b_g,k-1 + b_g,k))
r_g = wrap_SO2(yaw_k - yaw_k-1 - Delta_yaw_m) / sigma_Delta
sigma_Delta = dt * hypot(sigma_g * r_scale_gyro,k, sigma_model_rate)
```

`gyro_increment_model_sigma` (`rad/s`) represents uncertainty in the zero-order-hold (backward Euler) approximation and unmodeled intra-interval acceleration. It is **not** a continuous-time PSD and must not be multiplied by `sqrt(dt)`.

The exact Jacobians away from the SO(2) branch cut are:

```
d r / d x_k-1: yaw=-1/sigma_Delta, bg=+0.5*dt/sigma_Delta
 d r / d x_k:  yaw=+1/sigma_Delta, bg=+0.5*dt/sigma_Delta
```

**Critical no-double-count rule:** if the increment factor is constructed for an accepted IMU measurement, its original unary `GyroCost` is **omitted**. If dt is invalid, very small, or exceeds the configured bound, only the legacy unary gyro factor is constructed. Subsequent transitions use new gyro measurements; the previous measurement is not reused for trapezoidal gyro *rate* integration. The **bias** is midpoint averaged because bias states already exist at both endpoints.

This factor connects **adjacent** states only. The retained graph and block-tridiagonal Schur prior remain valid; introducing direct non-neighbor links would require redesigning marginalization.

## Limitations

- This is a zero-order-hold quadrature of an **instantaneous gyro rate measured at interval end**, not true high-frequency IMU preintegration. If angular acceleration is large, quadrature error is approximately `0.5 * angular_acceleration * dt**2`. The model-rate sigma softens that approximation; it cannot remove systematic lag.
- An integrated gyro factor supplies *relative yaw information*; it **cannot make absolute yaw observable**, nor can it separate bias, slip, and wheel calibration without adequate excitation/independent information.
- The process yaw factor and gyro increment can both be present: process is a motion prior, gyro is a measurement likelihood. No second use of the same gyro observation is allowed.
- Covariance/NEES may change. A/B on identical replay data against Gazebo ground truth is necessary before deployment. Success in numerical unit tests does not imply lower Gazebo yaw RMSE.

## Parameter flags

```
solver:
  gyro_increment_factor_enabled: true  # Gazebo experiment only
  gyro_increment_model_sigma: 0.12    # rate uncertainty [rad/s]
  gyro_increment_max_dt: 0.030       # s, otherwise unary fallback
```

In `config/mhe.yaml` and `config/mhe_user_baseline.yaml` the feature remains **off**. Run the Docker smoke A/B using `ci/run_gyro_increment_ab.sh`. `graph_status` appends `[19] enabled`, `[20] active factors`, `[21] max dt`, `[22] model-rate sigma` without changing old indices.

## Validation

- `benchmark/test_gyro_increment.cpp`: analytical Jacobian vs centered finite differences; cyclic angle wrapping; invalid intervals.
- `benchmark/benchmark_gyro_increment.py`: synthetic discrete-time noise envelope at 40/55/100 Hz; reports but cannot validate Gazebo yaw.
- `ci/test_gyro_increment_config.py`: safe defaults and source/config wiring.
- `ci/run_gyro_increment_ab.sh`: launches **the actual ROS 2/Ceres node** twice in Docker with the same type of **synthetic** sensor stimuli. It is not proof of improved localization.
- Run physical/Gazebo held-out rosbag replay and check yaw RMSE, P95, NEES, rollback, callback P99, `map->odom` corrections, and TF continuity. Do not accept a change solely because synthetic error is small.
