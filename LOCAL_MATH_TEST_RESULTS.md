# Local offline test report — not ROS/Gazebo evidence

| Test | Result |
|:--|:--|
| `benchmark/test_scaled_covariance.cpp` | PASS, 248 checks |
| `benchmark/test_se2_error_correction.cpp` | PASS, 9,007 checks |
| `benchmark/test_block_schur.cpp` | PASS, 864 Schur/dense comparisons |
| `benchmark/test_fast_covariance.cpp` | PASS, 13 worker checks |
| `python3 -m unittest discover -s ci -p test_*.py -q` | PASS, 48 tests |
| `benchmark/run_benchmarks.sh` | PASS, all offline regression suites |
| `ci/check_regressions.py` | PASS, synthetic regression checks |
| `benchmark/benchmark_covariance_math_ab.cpp` | PASS, synthetic kernel A/B |
| `colcon build` (Kilted, ROS2 Ceres) | NOT RUN — ROS 2 unavailable locally |
| `docker` real ROS node CI | NOT RUN — Docker/ROS 2 unavailable locally |
| Gazebo simulated ground truth test | NOT RUN — user's Gazebo not accessible |
| Physical robot | NOT RUN |

Paired 117-column synthetic covariance kernel, 120 trials, single host run:
- legacy p50 **0.295936ms**, p99 **1.17156ms**
- Jacobi-scaled LLT p50 **0.631211ms**, p99 **1.79221ms**

The new covariance method is not faster in this test; it introduces numerical safeguards. It is opt-in.

Implementation caveats: Ceres/ROS compile not verified here. LLT rcond is an approximate reciprocal condition estimate. Full-rank SVD fallback rejects singular windows rather than returning a nullspace pseudoinverse covariance. The SE2 pose correction remains experimental and off by default, and does not replace encoder/IMU extrinsic or timing calibration.
