# Local verification report — 2026-10-08

## Completed in the provided execution environment

- Existing plus new `benchmark/run_benchmarks.sh`: **PASS** (all existing offline regressions, analytic Jacobian reference, new 144-case block Schur numerical comparison).
- `python3 -m unittest discover -s ci -p 'test_*.py' -v`: **34 Python tests PASS**.
- `ci/check_regressions.py`: **PASS** (including new Schur-vs-dense correctness threshold).
- `ci/test_ci_harness.py`: **PASS** (positive and negative regression-gate tests).
- `benchmark/test_source_invariants.py`: **PASS**.
- YAML: **3 config files parse** and the new settings are nested under `solver`.
- Shell `bash -n` and Python `py_compile`: **PASS**.
- Synthetic NumPy arithmetic benchmark: **144** prefix-elimination cases; max absolute Hessian difference `3.469446951953614e-18`, max absolute RHS difference `2.220446049250313e-16`.

## Not run / do not overinterpret

- ROS2 Kilted/Ceres compilation and C++ CTest binaries (`test_incremental_ceres`, `test_block_schur`) **NOT RUN** here: ROS2, Ceres and Eigen are absent from this execution environment.
- Docker build and `run_ci.sh` **NOT RUN** here: Docker unavailable.
- No actual Gazebo or real robot run; no grounded claim of yaw accuracy improvement or Ceres speedup.
- Python chain-Schur median in these small examples was **slower** than NumPy dense Schur. This is not a C++ speed result.

## Run on the user's ROS2 machine

```
colcon build --packages-select mhe_sensor_fusion
```

For full CTest, actual ROS node smoke, sequential A/B and 25s fault stress:

```
docker build -f docker/Dockerfile.ci -t mhe-fusion-ci .
mkdir -p ci-results
docker run --rm --network host -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e CI_ARTIFACT_DIR=/artifacts -v "$(pwd)/ci-results:/artifacts" mhe-fusion-ci
```

See `INCREMENTAL_GRAPH_SCHUR.md` for diagnostic indices and interpretation.
