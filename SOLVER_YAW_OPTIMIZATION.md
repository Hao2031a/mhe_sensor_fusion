# MHE solver / yaw follow-up (Gazebo)

## Changes

* **QR retry is conditional.** QR fallback from Normal Cholesky is only attempted
  for numerical or cost failures. Physically implausible endpoint jumps skip QR
  by default, instead of doing a second complete solve that cannot repair the
  endpoint motion constraint. Optional `solver.qr_fallback_on_endpoint_jump`
  (default `false`) keeps a reversible debug escape hatch.
* **Shared soft deadline.** QR reuses only the leftover time from
  `solver_budget_ms`; `solver.qr_fallback_min_remaining_ms` (default 1.0 ms)
  prevents starting a retry with negligible time remaining. **This does not
  guarantee WCET**: Ceres time limits are soft, graph-build/covariance work
  are not included in the Ceres solve time limit, and the ROS executor can
  still miss a 10 ms callback deadline.
* **Diagnostics** `/mhe/solver_health` preserves original indices 0..10 and
  appends [11] final rejection reason (`0` accepted, `1` numerical/nonfinite,
  `2` cost increase, `3` implausible endpoint), [12] cumulative QR fallback
  attempts, [13] cumulative skipped retries. `/mhe/rt_profile` remains stable.
* Gazebo low-latency YAML keeps `output.smoothing.angular_enabled: false`, and
  uses the solver thresholds the user supplied (20 states, 40 rad/s²,
  0.5 m/s, 1.5 rad/s); these are **loose acceptance thresholds**, not validated
  as the best values. `config/mhe_user_baseline.yaml` is preserved unchanged.

## Run on Ubuntu with ROS 2 Kilted

```bash
colcon build --packages-select mhe_sensor_fusion
ros2 launch mhe_sensor_fusion mhe_gazebo_low_latency.launch.py
ros2 topic echo /mhe/solver_health --once
ros2 topic echo /mhe/rt_profile --once
```

Ensure no other MHE/robot_localization node publishes `/odom` or `odom -> base_footprint`.
Gazebo and the MHE node must both use the simulation clock.

## Benchmark and CI

```bash
bash benchmark/run_benchmarks.sh
# With Docker available:
docker build -f docker/Dockerfile.ci -t mhe-fusion-ci .
mkdir -p ci-results
docker run --rm --network host \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e CI_ARTIFACT_DIR=/artifacts \
  -v "$(pwd)/ci-results:/artifacts" mhe-fusion-ci
```

Unit tests check rejection classification, time sharing, opt-in endpoint retries,
NaN/negative budgets, and retain yaw bypass regression tests. The *synthetic*
benchmark quantifies changed **fallback policy decisions**, not actual Ceres
latency or actual Gazebo yaw accuracy. The Docker integration smoke tests the
real ROS node on **synthetic sensor inputs**. Run ground-truth Gazebo A/B tests
of heading error and 100-Hz callback P99 before production use.

## How to interpret the existing solver parameters

With `solver.prefer_normal_cholesky: true`, the implementation always starts
with normal Cholesky, so changing `solver.dense_qr_max_states` alone does not
switch the first-pass linear solver. Set `prefer_normal_cholesky: false` to let
that threshold select Dense QR for small horizons. The current Gazebo YAML
retains the user's threshold of `20` but still prefers Cholesky.

`solver.cost_tolerance` is **relative**: `final_cost <= initial_cost *
(1 + tolerance) + 1e-9`. The endpoint guard checks
`|v_new-v_ref| <= a_max * clamp(dt,0,0.25) + linear_jump_margin`, likewise for
angular velocity. The user's angular acceptance values 40 rad/s² and 1.5 rad/s
are quite permissive and could accept real endpoint spikes; they were not
re-tuned without ground truth.

## Validation scope

Local environment has **no ROS 2 / Docker**, so **no colcon build, Docker CI,
actual Ceres runtime or Gazebo simulation was executed here**. Source-level
regression checks and offline compiler/unit benchmarks passed. Next validation
must compare real `/mhe/rt_profile` callback P95/P99 and
`/mhe/solver_health` fallback/rollback counts before and after, then Gazebo
pose/yaw compared against ground truth at matched simulation timestamps.
