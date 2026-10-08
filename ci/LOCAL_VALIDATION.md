# CI implementation validation: executed in artifact-building environment

The following are **actual local results**, not predictions:

- `bash -n ci/run_ci.sh benchmark/run_benchmarks.sh` — PASS.
- `python3 -m py_compile ci/*.py benchmark/*.py` — PASS.
- GitHub workflow YAML syntax and MHE configuration YAML parse — PASS.
- `g++ -O2 -DNDEBUG ... benchmark/test_stability_guards.cpp` — PASS: 12 checks.
- `./benchmark/run_benchmarks.sh` — PASS, synthetic models and source-invariant checks.
- `python3 ci/check_regressions.py` — PASS, nine correctness thresholds.
- `python3 ci/test_ci_harness.py` — PASS, forced failure exits nonzero.

**Not executed in the artifact-building environment:** Docker image pull/build;
`colcon build` / `colcon test` under Kilted; runtime rclpy publishers and the
actual Ceres MHE node integration smoke test. This container does not have
Docker or ROS installed. Those steps are defined as blocking commands in the
GitHub Actions job and must be verified there before merging production code.

Performance p50/p95/p99 are recorded in `ros_smoke.json` after the container
runs. Absolute timing performance is not compared across shared GitHub hosted
runners; use dedicated hardware and a controlled workload for hard timing gates.
