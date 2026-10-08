# Local validation — rank-aware mathematical optimization (2026-10-08)

Source package: `mhe_sensor_fusion` (no version suffix).

## Verified locally, standalone mathematical kernels (not ROS)

- C++ rank-aware square-root Hessian tests: 1,354/1,354 PASS; ranks 1 through 9, and checks for inconsistent nullspace gradient and indefinite Schur matrix; max relative H reconstruction `4.9959e-15`; max gradient `2.98704e-15`.
- C++ Chain-Schur -> rank-aware prior: 672/672 PASS, max relative error `4.93535e-15`.
- NumPy Monte Carlo rank-aware reference: 360/360 PASS; max H relative error `2.8601e-15`, max gradient relative error `2.1021e-15`.
- Python unit tests: 38/38 PASS.
- Existing offline benchmark suite: PASS, including yaw latency, QR policy, covariance consistency reference, Schur and factor selection.
- CI regression gate: PASS, includes rank-aware Hessian/gradient checks.
- Three YAML files parsed and configurations checked for opt-in/rollback; shell scripts parsed.

## Performance honesty

- C++ O2 isolated 9x9 numerical kernel (12 repeats × 128 Hessians): old eigenvalue floor + LLT median **4,721 ns/prior**; new rank-aware eigendecomposition median **4,915 ns/prior**, ratio **1.041** (new ~4.1% slower on this microbenchmark). Values depend on shared container CPU and are not RT-certified.
- The improvement is **mathematical correctness when information is rank deficient**, not proven throughput.
- Existing NumPy block Schur timing also does not show a win for small horizon and is not representative of the ROS/Ceres node.

## Not run

- Complete CMake/colcon ROS 2 Kilted compilation (ROS/Ceres unavailable locally).
- Docker build / Ceres actual node A/B CI (Docker unavailable locally).
- Gazebo ground-truth yaw, NEES, TF continuity and physical robot.

## Regression/rollback

- Standard `config/mhe.yaml` remains `solver.rank_aware_prior_enabled=false`.
- Gazebo experimental `config/mhe_gazebo_low_latency.yaml` sets true.
- A/B Docker script `ci/run_rank_prior_ab.sh` compares full Ceres nodes with scripted synthetic sensor input and writes profile JSON artifacts.
- Any runtime solver rollback, invalid-prior counter increase, yaw NEES regression or slowdown warrants switching this parameter back to false pending analysis.
