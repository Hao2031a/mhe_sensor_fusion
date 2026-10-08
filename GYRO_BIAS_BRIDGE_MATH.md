# Opt-in gyro-bias Brownian-bridge mathematical correction

## Model
If the gyro bias is a Brownian random walk `db_g(t)=q_b dW_t`, then
conditional on its two endpoint values, the remaining path noise is a
Brownian bridge. The integral of its covariance kernel
`q_b^2(min(s,t)-st/dt)` over both time variables yields exactly:

`Var[integrated bias | b_g(0), b_g(dt)] = q_b^2 dt^3 / 12`.

The increment factor currently approximates constant rate using one gyro
sample, with rate-measurement standard deviation sigma_g and a separate
quadrature mismatch sigma_m (rad/s). The new variance becomes:

`sigma_angle^2 = dt^2(sigma_g^2+sigma_m^2)+q_b^2 dt^3/12`.

The process factor uses `sigma_bg(dt)=sigma_bg_nom sqrt(dt/T0)`,
so the consistent diffusion is `q_b=sigma_bg_nom/sqrt(T0)`;
the code derives it from the same process noise, with `T0=1/frequency`.
The gyro increment still uses **one gyro measurement exactly once**.
Its SO(2) residual, analytic Jacobians, and factor sparsity are unchanged.

## Use and limitations
`solver.gyro_increment_bias_bridge_enabled: false` is the default.
For A/B use `config/mhe_gazebo_bias_bridge_ab.yaml`; it enables the
variance term but keeps the rest of low-latency Gazebo settings.
When gyro increments are disabled or too old, the ordinary gyro factor
still applies. New /mhe/graph_status entries: [23] enabled, [24] q_b.

Current default bias process sigma 0.002 per nominal 10ms gives a small
variance correction relative to model sigma 0.12 rad/s. This improves
noise-model consistency **but does not imply better yaw RMSE**.
It is not multi-sample IMU preintegration and does not correct timestamps.

## Verification
- C++ test_gyro_increment: finite-difference Jacobians (63k checks),
  exact A/B legacy case q_b=0, variance and invalid-parameter guards.
- benchmark_gyro_bias_bridge.py: seeded stochastic bridge variance test.
- ci/run_gyro_bias_bridge_ab.sh: actual ROS 2/Ceres node with synthetic
  IMU and encoder messages via Docker CI (not Gazebo).
Validate yaw RMSE, gyro NIS and TF timing against real recorded ground truth.
