# Full-Horizon Nuisance-Profiled Observability

The MHE state is [x,y,yaw,v,w,b_g,b_a,sL,sR]. This module analyzes the
wheel and rate-gyro measurement Jacobians of **every state in the horizon**.

It holds slow perturbations (delta b_g, delta s_common, delta s_diff) shared
across the window; each timestamp has its own nuisance (v_k,w_k). The process
random walk on v,w couples the nuisance blocks at adjacent states.

From J=[A B], the projected Fisher information is

    F = B^T B - B^T A (A^T A)^-1 A^T B

The A^T A nuisance block is block-tridiagonal. O(N) block LDLT and a 2x2
Moore-Penrose profile over the other target modes give per-parameter scores
score_j = 1 - exp(-I_j / information_reference). No arrival prior, wheel
slip prior, or artificial eigenvalue floor enters the observability estimate.
The score is used ONLY to gate later slip-prior weights, never to add another
sensor measurement. No pose/yaw/TF low-pass smoothing was added.

IMPORTANT: This is the *relevant reduced subspace* of the full 9-state
horizon, not a complete 9N eigenanalysis, and its gyro-rate rows are a proxy
when the node runs interval gyro-increment factors. It does not make absolute
yaw observable without an independent heading reference. Data-conditioned
prior variance is an empirical-Bayes heuristic, not exact posterior covariance.

New parameters: solver.horizon_observability_enabled (default false),
update_every_n=5, min_wheel_samples=5, min_gyro_samples=5,
min_span_sec=.08, information_reference=.25, process_weight_scale=1,
bias_scale=.03, slip_scale=.02. All have the horizon_observability_ prefix.

A/B YAML: config/mhe_gazebo_horizon_observability_ab.yaml (enabled).
The baseline YAML remains unchanged in behavior.

Topic /mhe/horizon_observability is Float64MultiArray:
[0] enabled, [1] valid, [2] states, [3] wheel samples, [4] gyro samples,
[5] span, [6] rank, [7:9] information for bg, common/diff slip,
[10:12] scores, [13] kernel ms, [14] cumulative updates.

CI builds C++ tests and compares 900 synthetic horizons with an independent
dense Jacobian/Schur elimination. ROS2 Kilted/Ceres A/B runs the real estimator
with synthetic wheel+IMU messages. Gazebo/physical yaw RMSE is not yet tested.
