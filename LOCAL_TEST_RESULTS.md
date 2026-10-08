# Local verification — timestamp alignment + yaw feedback

Execution environment: g++/Python available; ROS 2, Docker, and Eigen headers
are **not installed**. **No colcon build, Ceres solve benchmark, Gazebo session,
real robot, or actual ROS smoke integration was executed here.**

## PASS offline

- `bash benchmark/run_benchmarks.sh`: all built-in offline regression
  benchmarks, source invariants, compiled low-latency yaw tests, compiled
  timestamp/pause/clock-jump tests, and 30-seed yaw synthetic policy test PASS.
- `python3 -m unittest discover -s ci -p 'test_*.py'`: 32 Python tests PASS.
- `g++ -O2 -DNDEBUG ... benchmark/test_stability_guards.cpp`: 26 checks PASS.
- `g++ -O2 -DNDEBUG ... benchmark/test_rt_budget.cpp`: 15 tests PASS.
- Standalone `test_time_aligned_output.cpp`: PASS, including first tick,
  exact dt, pause, backward clock jump, forward gap, future event and stale data.
- Standalone `test_low_latency_output.cpp`: PASS, including trusted endpoint,
  confidence, gyro NIS, covariance, wrap-around, timestamp age, outlier and rate gates.
- `ci/check_regressions.py`: PASS with newly required timestamp-yaw report.

## Synthetic yaw policy benchmark

Synthetic conditions only: 30 seeds, 100 Hz output, 55 Hz synthetic reference,
independent simulated reference yaw noise, deliberate +0.012 rad/s drift in
propagated rate and occasional +0.8 rad outliers. The *Python policy model*,
not the ROS/Ceres implementation, produced:

- unanchored mean yaw RMSE: 0.11453298597 rad
- rate-limited correction mean yaw RMSE: 0.00090578794 rad
- worst correction step: 0.00123246647 rad/tick
- 30/30 seeds satisfied the defined synthetic regression checks.

**Do not infer Gazebo/real sensor accuracy improvements from these numbers.**
The source artifact and Docker CI scripts include runnable ROS node tests, but
those must be executed by the user in a ROS 2 Kilted + Gazebo environment.

## Required before deployment

1. `colcon build --packages-select mhe_sensor_fusion`.
2. Run Docker CI, inspect `ci-results/ros_smoke.json` and RT/long-stress gates.
3. On Gazebo compare stage-1 only (`yaw_feedback.enabled=false`) vs stage-1+2
   (`yaw_feedback.enabled=true`) with independent ground truth and identical
   maneuvers. Audit `/mhe/time_alignment` and `/mhe/rt_profile`.
4. Verify `/odom` and `odom -> base_footprint` have no duplicate publishers.


## Analytic Factor Optimization (this version)

- 1,200 randomized process/wheel factor cases: 426,000 finite-difference
  Jacobian, reference residual, partial-Jacobian and no-Jacobian checks PASS.
- Python unit tests: **34/34 PASS** (including YAML solver namespace tests).
- `benchmark/run_benchmarks.sh`: PASS.
- `ci/check_regressions.py`: PASS.
- All 3 YAML profiles successfully parsed with expected analytic A/B flags.
- Not run here: Ceres-linked `test_ceres_analytic`, ROS/Kilted colcon build, Docker
  ROS node smoke or Gazebo ground truth (ROS/Ceres not available).

The analytic Jacobian is an exact derivative of the *same factor*, not a new
trajectory model. Actual speedup remains to be measured in Docker CI; no
Ceres runtime improvement percentage is claimed from offline tests.
