# Offline synthetic benchmark — robustness/scalability upgrade

This is a model-level benchmark, not a ROS/Gazebo/Ceres runtime benchmark on the target PC. It is intended to validate the direction of the four changes before on-robot testing.

## Common-mode slip

Scenario: true common slip `sC = 0.20`, dynamic acceleration from 1–4 s, then constant-speed motion. Synthetic wheel acceleration follows `(1+sC)*a_true` with noise; IMU acceleration has independent noise.

- old constant-speed/unobservable baseline: approximately `0`
- new peak estimate during excitation: `0.270`
- estimate at end of excitation: `0.159`
- mean retained estimate, 4–6 s: `0.139`
- mean retained estimate, 6–8 s: `0.108`

The new estimator therefore learns a substantial fraction of common slip during excitation and retains it into the later unobservable interval. It still cannot determine absolute common slip from scratch during indefinitely constant straight motion without an independent ground-speed source.

## Accelerometer robustness

Scenario: longitudinal channel contains a `+9.81 m/s²` static projection, `0.08 m/s²` Gaussian noise, and several ±6 m/s² single-sample spikes.

- old LPF-only RMSE: `9.787 m/s²`
- auto-zero + LPF + jerk-limit RMSE: `0.104 m/s²`
- improvement factor: about `93.8×`
- learned static offset: `9.788 m/s²`

This directly targets the previously observed ~9.9 m/s² residual failure mode.

## Covariance consistency

A scalar Gaussian least-squares Monte-Carlo check compares posterior variance from the inverse Hessian with empirical estimator variance.

For 5 measurements (`σ=0.1`):

- empirical variance: `0.002023`
- Hessian prediction: `0.002000` (~`1.12%` relative error)
- fixed variance `0.01`: ~`394%` relative error

For 20 measurements:

- empirical variance: `0.0004968`
- Hessian prediction: `0.0005000` (~`0.64%` relative error)
- fixed variance `0.01`: ~`1913%` relative error

This shows why information-dependent posterior covariance is preferable to a fixed heuristic covariance.

## Horizon scalability

For 100 Hz IMU + 50 Hz wheel odometry:

- old per-event solve upper rate: ~`150 solves/s`
- new batched solve cap: `100 solves/s`
- nominal solve-count reduction: ~`33.3%`

For the observed burst where `/mhe/timing[0] = 4` events arrive in one output tick:

- old: up to 4 solves in that tick
- new: 1 solve
- solve-count reduction in that burst: `75%`

Multi-state prefix marginalization also allows all expired states from a burst to be removed in one Schur operation rather than allowing the horizon to grow.
