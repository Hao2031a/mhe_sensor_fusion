# MHE Low-Latency Yaw Output for Gazebo (Experimental)

## Problem and change

When `output.smoothing.enabled=true` in the original estimator, **both** v and
omega pass through a median-of-three + adaptive first-order filter + acceleration
and jerk limit. The published `odom->base_footprint` pose is then integrated from
these output velocities rather than snapped to each nonlinear Ceres solution.
A transient delay in omega therefore integrates into a yaw error visible in RViz.
The user independently observed a large scan/map alignment improvement with the
original master smoothing switch OFF on Gazebo.

New independent switches allow **linear-only smoothing**:

```yaml
output:
  smoothing:
    enabled: true
    linear_enabled: true
    angular_enabled: false
  yaw_feedback:
    enabled: false
```

- Angular bypass applies NO median, LPF, acceleration or jerk shaping.
- Linear smoothing remains enabled to reduce translational velocity chatter.
- Master `enabled: false` STILL bypasses both channels exactly as before.
- Deadbands, stationary zero-velocity and stale-sensor guards still apply.
- The MHE Ceres objective, window, wheel/IMU model and TF frame tree are unchanged.
- `output.yaw_feedback.enabled=false` by default: the optional correction is
  gated by accepted Ceres solve and timestamp age; capped to `max_rate` rad/s
  and `max_error` radians to avoid abrupt jumps. It is **NOT** recommended until
  Gazebo independent ground truth confirms persistent yaw drift. Applying it
  without truth can make pose-vs-IMU inconsistencies worse.
- `/mhe/output_status` indices 0..15 remain unchanged. Appended:
  16: linear smoothing configured, 17: angular smoothing configured,
  18: last yaw-feedback error rad, 19: last feedback correction rad,
  20: feedback applied counter, 21: feedback enabled.

## Baseline and candidate

* `config/mhe_user_baseline.yaml`: all uploaded user parameter settings retained,
  with only the original package's **default-disabled** opt-in covariance
  calibration settings carried over.
* `config/mhe.yaml`: new hybrid output plus a 6-ms Ceres soft budget and 4.5-ms
  covariance snapshot scheduling budget (the user's most recent file specified
  12 ms for each; the callback target period is 10 ms). The hard deadline is
  NOT guaranteed merely by selecting a budget. Other user settings including
  `window_size: 15`, `sensor_timeout: 0.2`, `accel.gravity_compensation: true`,
  `transform_time_offset: 0.0` are kept.
* `config/mhe_gazebo_low_latency.yaml`: the candidate plus `use_sim_time: true`.

**Do not launch an additional MHE alongside an existing localization launch**:
only one publisher must own `/odom` and `odom->base_footprint`. If your
`diffbot_localization` launch already owns MHE, point its `parameters` to the
candidate YAML (do not leave it using `ekf.yaml` unless it contains correct MHE
parameters). Inspect effective ROS parameters after launch.

### Run

```bash
colcon build --packages-select mhe_sensor_fusion
ros2 launch mhe_sensor_fusion mhe_gazebo_low_latency.launch.py
ros2 param get /mhe_sensor_fusion use_sim_time
ros2 param get /mhe_sensor_fusion output.smoothing.enabled
ros2 param get /mhe_sensor_fusion output.smoothing.linear_enabled
ros2 param get /mhe_sensor_fusion output.smoothing.angular_enabled
ros2 topic hz /odom
ros2 topic echo /mhe/rt_profile --once
ros2 topic echo /mhe/output_status --once
```

A clean restart of the source overlay is required; rebuilding a package does
not modify an already running executable. The `rt_profile` topic must exist
for this version. Confirm simulator's independent ground-truth pose topic
before attributing residual drift to MHE.

### Validate yaw against independent Gazebo truth

For an independent simulator-pose topic bridged as `nav_msgs/msg/Odometry`:

```bash
python3 ci/evaluate_gazebo_yaw.py \
  --truth-topic /gazebo/ground_truth_odom \
  --estimate-topic /odom --duration 60 \
  --output gazebo_yaw_candidate.json
```

`/gazebo/ground_truth_odom` is an **illustrative placeholder**: inspect your
simulation's real ground-truth topic first, and bridge the *model pose from
Gazebo*, not the wheel odometry. Run an equal trial with the baseline config.
The script interpolates yaw by ROS timestamps, unwraps ±pi and subtracts the
initial offset. Report RMSE, p95 and maximum relative yaw error; also log
`/tf`, `/scan`, `/mhe/rt_profile`, and `/mhe/solver_health`. Compare at equal
simulated speeds, routes and seed, and check Gazebo real-time factor.

### Synthetic benchmark and CI

```bash
bash benchmark/run_benchmarks.sh
python3 -m unittest discover ci -p 'test_*.py' -v
# Full ROS 2 + Ceres integration: requires Docker and ROS 2 installation.
docker build -f docker/Dockerfile.ci -t mhe-fusion-ci .
mkdir -p ci-results
docker run --rm --network host \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e CI_ARTIFACT_DIR=/artifacts \
  -v "$(pwd)/ci-results:/artifacts" mhe-fusion-ci
```

The 30-seed yaw latency test models a 55-Hz synthetic rate sensor and a 100-Hz
publish tick, with identical hold and noise for original shaped angular output
and bypass; it reports relative yaw RMSE and p95 latency error. It is **NOT a
Gazebo experiment, NOT independent physical ground truth, and NOT a direct ROS
node performance measurement**. Existing RT and long-stress tests remain in
Docker. Python C++ offline tests cannot prove ROS startup/build succeeds.

## Next optimization gate

Do not activate feedback or change `process.sigma_w`, `gyro_z_scale`, or
`wheel_separation` until the independent Gazebo yaw trace reveals whether the
remaining error is transient lag, sustained bias or a `map->odom` scan-match
correction. Note scan_time=0 in Gazebo gives no proof of spinning-laser motion
distortion; don't deskew without per-ray timestamp evidence.
