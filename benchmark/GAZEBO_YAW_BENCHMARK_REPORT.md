# Yaw latency benchmark — synthetic, not Gazebo

Source: `benchmark/benchmark_yaw_latency.py` (30 deterministic seeds)

| Metric | Previous angular shaping | Angular bypass (hybrid) |
|---|---:|---:|
| Yaw RMSE, rad | 0.05029 | 0.00524 |
| Absolute yaw error p95, rad | 0.08930 | 0.00930 |
| Reduction | — | 89.59% RMSE; 89.59% p95 |

Scenario: **synthetic** 55-Hz sensor sample-and-hold with Gaussian angular
noise standard deviation 0.003 rad/s, 100-Hz output ticks, start/stop/reverse
piecewise-constant angular maneuvers. Baseline models the original production
median-of-three, adaptive low-pass and jerk/acceleration shaping; candidate
bypasses angular shaping. This is a focused latency/isolation test, not a
full Ceres MHE, Gazebo, LiDAR scan matcher, DDS, or TF benchmark. Result must
be confirmed with actual ROS node/Gazebo ground truth. No claim of 89.59%
real-world map improvement is implied.

Regression gate: at least 40% reduction in each error metric and candidate
RMSE below 0.025 rad. The reference/noise is intentionally identical in both
cases and the noisy sensor rates are held constant.

Tests in this artifact: `benchmark/run_benchmarks.sh` => PASS,
`ci/check_regressions.py` => PASS, `python -m unittest discover ci` 32 tests PASS,
`benchmark/test_low_latency_output.cpp` compiles/runs PASS.

Full ROS 2/Kilted Ceres binary build, Gazebo runtime and Docker integration:
**not executed in the artifact-generation environment** (no ROS/Docker).
