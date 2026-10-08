# MHE hot-path code review: low-latency Gazebo profile

This is an incremental change to **`mhe_sensor_fusion`**. All ROS topics and the 9-state estimator are retained. The previous low-latency yaw optimization remains in place: angular smoothing is bypassed by default and optional bounded yaw feedback stays disabled by default.

## Implemented changes

1. **Compact inbound events.** The callbacks extract precisely the ROS Odometry/IMU fields used by the estimator. Each queue element stores compact scalar payloads (`CompactOdom` = 64 bytes, `CompactImu` = 72 bytes in the tested C++ ABI), not BOTH complete ROS messages. Full ROS messages are no longer copied into the event queue. The event sequence and timestamp are retained.
2. **Reusable bounded sort buffer.** `drainSensorEvents()` reuses a member `std::vector` across 100-Hz ticks instead of allocating a new vector each time. `std::sort` orders `(timestamp, sequence)` deterministically; the former stable sort was unnecessary because sequence IDs break ties.
3. **Reduced Ceres allocation churn.** Reuse two immutable `HuberLoss` objects within each Ceres `Problem` (one for wheel/gyro/common-slip and one for acceleration) rather than allocating an individual loss per factor. Ceres `Problem` owns/deduplicates shared loss pointers. Cost functions, robust loss parameters, solver type, and noise models are unchanged.
4. **Diagnostics decoupled from odometry.** New `diagnostics.frequency` (default `25.0` Hz, also explicit in the standard and Gazebo YAML) rates limits 15 informational topics. **`/odom`, its TF, `/mhe/rt_profile`, and `/mhe/cov_worker_status` remain on the unthrottled output-timer path**. On an ideal 100-Hz timer, this saves 75% of informational-topic publications: 1500/s => 375/s. This is a *count estimate*, not a measured CPU or DDS throughput benchmark.
5. **Gazebo `/clock` rewind hygiene.** When ROS time jumps backward the estimator clears any queued pre-reset sensor events, in addition to its previous state reset. `/mhe/timing` adds a **13th** value (`index 12`), cumulative queued events flushed by ROS-time rewinds. All existing indices 0..11 are retained.
6. **More accurate real-time profiling.** Full callback duration and deadline misses are measured **after** both real-time profile/worker-status publications. `/mhe/rt_profile` therefore reports the callback duration of the *previous completed tick*, not a truncated measurement of the current tick. No changes to array indices.

## What was deliberately NOT changed

- No alternative kinematic model, no changes to encoder/IMU measurement weights or slip estimation, no EKF or scan matching adjustments.
- No extra pose feedback or time correction on the published yaw; `output.yaw_feedback.enabled` remains `false`.
- No ROS topic renaming, no frame changes, no extra `map -> odom` TF; no modification to the optional unexercised features.
- Ceres graph creation remains per solve because caching a `Problem` across deque state expiration and marginalization requires extensive ownership/linearization correctness work; it is unsafe to make that alteration without ROS/Gazebo integration tests.

## Expected Gazebo behavior

- Use `config/mhe_gazebo_low_latency.yaml` as the optimized variant: `use_sim_time: true`, `transform_time_offset: 0.0`, `output.smoothing.linear_enabled: true`, `output.smoothing.angular_enabled: false`.
- Use `config/mhe_user_baseline.yaml` for the prior user's YAML and a controlled baseline (note that the **new source-level** diagnostic default applies even to the baseline unless overridden).
- If disabling *all* smoothing worked best on your robot, leave `output.smoothing.enabled: false` in the selected YAML. `angular_enabled: false` already protects yaw from angular smoothing, but linear smoothing may affect the timing of translational movements.
- The odometry/IMU input rate of ~55 Hz does not automatically imply the output must be 55 Hz; the observed wall-clock rate also depends on `/clock` increments and Gazebo real-time factor. A wall timer cannot synthesize unique simulation timestamps while `/clock` is frozen.

## Validation

Offline validations in this environment:

- `bash benchmark/run_benchmarks.sh` -> PASS. It includes the 30-seed **synthetic-only** yaw latency regression and all baseline synthetic covariance/innovation tests.
- `python3 -m unittest discover -s ci -p 'test_*.py'` -> 32 tests PASS.
- Six stand-alone C++ test binaries under `-fsanitize=undefined` -> PASS (including the fast covariance worker; does not compile the ROS node).
- All three YAML configs parse successfully.

**NOT run here:** `colcon build`, ROS 2 Kilted smoke/integration tests, Gazebo odometry/TF/yaw ground-truth benchmark, Docker image and 600s physical/ROS long stress. These remain REQUIRED before accepting a real performance claim.

## Local commands

```bash
colcon build --packages-select mhe_sensor_fusion
ros2 launch mhe_sensor_fusion mhe_gazebo_low_latency.launch.py
ros2 param get /mhe_sensor_fusion diagnostics.frequency
ros2 param get /mhe_sensor_fusion output.smoothing.angular_enabled
ros2 topic hz /odom
```

If `diffbot_localization` already launches MHE, do **not** start another instance; update that launch's parameter file and restart.

Docker CI (from package root, where `docker/` exists):

```bash
docker build -f docker/Dockerfile.ci -t mhe-fusion-ci .
mkdir -p ci-results
docker run --rm --network host \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e CI_ARTIFACT_DIR=/artifacts \
  -v "$(pwd)/ci-results:/artifacts" mhe-fusion-ci
```

Stress run (synthetic stimuli on a real ROS process, **not** physical/Gazebo truth):

```bash
docker run --rm --network host \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e CI_ARTIFACT_DIR=/artifacts \
  -e MHE_STRESS_SECONDS=600 \
  -e MHE_STRESS_STRICT=1 \
  -e MHE_STRESS_CONSISTENCY_GATE=1 \
  -v "$(pwd)/ci-results:/artifacts" mhe-fusion-ci \
  /bin/bash /ws/src/mhe_sensor_fusion/ci/run_long_stress.sh
```

**Important:** A previous long-stress *synthetic covariance-consistency* gate failed even while RT deadlines passed. Do not describe the 600s gate as passing until rerun and investigated. Avoid blindly rescaling covariance just to make it pass.
