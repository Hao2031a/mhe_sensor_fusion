# Local offline verification — 2026-10-08

This verification was performed **without ROS2 Kilted / Ceres / Docker**.
It does not establish a Gazebo speedup or successful compilation of the full node.

- `python3 -m unittest discover -s ci -p 'test_*.py'`: **36 tests passed**.
- `bash benchmark/run_benchmarks.sh`: **exit 0**.
- C++ prefix candidate enumeration compiled with `-DNDEBUG`: **1770 exact set cases passed**.
- 9x9 block-Schur NumPy vs dense reference: **144 cases passed**;
  max absolute Hessian error `3.47e-18`, RHS error `2.22e-16`.
- Analytic process/wheel residual/Jacobian regression: **426000 checks passed**.
- Synthetic NumPy median: block path ~0.052 ms vs dense ~0.029 ms;
  **do not interpret this as C++ speedup**. Small-scale Python loop overhead
  dominates this synthetic benchmark.
- The node is still rebuilt/solved in Ceres on every active solve when the
  persisted graph is used; the work avoided is factor construction and
  irrelevant factor enumeration during marginalization.

Required on user's Gazebo/Kilted machine: `colcon build --packages-select mhe_sensor_fusion`,
Docker CI including Ceres CTests and ROS smoke, then matched rosbag A/B replay.
