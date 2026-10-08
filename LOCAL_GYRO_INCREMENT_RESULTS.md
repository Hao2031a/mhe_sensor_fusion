# SO(2) gyro increment — validation log

Scope: **offline only**. The container has `g++`, Python, NumPy and YAML but **no ROS 2 Kilted, Ceres headers, or Docker executable**. Do not interpret these tests as ROS node build or Gazebo navigation validation.

Validated:

- Standalone `benchmark/test_gyro_increment.cpp` compiled with `g++ -std=c++17 -O2 -DNDEBUG -Wall -Wextra -Werror`; **63,000** centered-difference Jacobian comparisons and cyclic angle/invalid-input checks passed.
- `benchmark/benchmark_gyro_increment.py`: **12** synthetic scenarios (40, 55, 100 Hz; angular acceleration 0/0.5/1.5/3 rad/s²), 25,000 samples each; synthetic test envelope passed. Note the modeled sigma `0.12 rad/s` is conservative and resulted in **100%** 95%-envelope coverage in this scenario — it is not covariance calibration evidence or proof of better yaw.
- `python3 -m unittest discover -s ci -p 'test_*.py'`: **44 tests passed**, including safe legacy defaults and no-double-use source wiring assertions.
- `benchmark/run_benchmarks.sh`, `ci/check_regressions.py`, and `ci/test_ci_harness.py`: PASS.
- `bash -n` checks for the new CI and benchmark scripts: PASS.
- YAML parsed with PyYAML: Gazebo experimental flag is true; legacy/default false.
- CMake adds `test_gyro_increment`. Docker CI `ci/run_ci.sh` includes CTest and executes `ci/run_gyro_increment_ab.sh`, which launches the real ROS node with synthetic measurements. **These Docker CI jobs have not been executed here**.

Risk / mandatory A/B on user's Gazebo:

1. Run same timestamped input bag twice: `solver.gyro_increment_factor_enabled=false` and `true`.
2. Compare yaw with *Gazebo ground truth*, measure RMSE and P95 both during acceleration/deceleration and steady turning.
3. Check `graph_status[20] > 0` in experimental mode (otherwise gyro delta factor never activated).
4. Compare full `/mhe/rt_profile`, rollback rate, solver residual, covariance NEES/coverage and TF continuity; reject change if metrics worsen.
5. If yaw drifts on sharp angular acceleration, decrease `gyro_increment_max_dt` and/or increase `gyro_increment_model_sigma`, or disable feature. No guarantee of improved yaw in every model.
