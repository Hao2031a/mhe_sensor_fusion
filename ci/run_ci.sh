#!/usr/bin/env bash
set -Eeuo pipefail
# ROS 2 generated setup.bash scripts read AMENT_TRACE_SETUP_FILES while unset.
# Temporarily disable nounset only while sourcing ROS and workspace overlays.
set +u
source /opt/ros/${ROS_DISTRO:-kilted}/setup.bash
source /ws/install/setup.bash
set -u

PKG=/ws/src/mhe_sensor_fusion
ARTIFACT_DIR=${CI_ARTIFACT_DIR:-/tmp/mhe-ci-artifacts}
mkdir -p "$ARTIFACT_DIR"

archive_logs() {
  set +e
  cp -a /ws/log "$ARTIFACT_DIR/colcon-log" 2>/dev/null || true
  cp -a "$PKG"/benchmark/*.json "$ARTIFACT_DIR/" 2>/dev/null || true
}
trap archive_logs EXIT

echo '::group::1. Colcon/CTest unit tests'
colcon test --packages-select mhe_sensor_fusion --event-handlers console_direct+
colcon test-result --verbose
# Explicitly enforce CTest exit status (some colcon versions only summarize failures).
(cd /ws/build/mhe_sensor_fusion && ctest --output-on-failure)
# Mathematical Schur prior correctness (rank 1..9, nullspace, indefinite guard).
/ws/build/mhe_sensor_fusion/test_gyro_increment
(cd "$PKG/ci" && python3 -m unittest -v test_gyro_increment_config)
/ws/build/mhe_sensor_fusion/benchmark_covariance_math_ab --output "$ARTIFACT_DIR/covariance_math_kernel_ab.json"
/ws/build/mhe_sensor_fusion/test_scaled_covariance
/ws/build/mhe_sensor_fusion/test_se2_error_correction
/ws/build/mhe_sensor_fusion/test_so2_yaw_math
/ws/build/mhe_sensor_fusion/test_ceres_so2_yaw
(cd "$PKG/ci" && python3 -m unittest -v test_so2_yaw_config)
/ws/build/mhe_sensor_fusion/test_rank_aware_prior
/ws/build/mhe_sensor_fusion/test_rank_aware_schur
/ws/build/mhe_sensor_fusion/benchmark_rank_aware_kernel \
  --output "$ARTIFACT_DIR/rank_prior_kernel_ab.json"
# Background worker benchmark under a 117-variable representative window.
/ws/build/mhe_sensor_fusion/test_fast_covariance \
  --benchmark-output "$ARTIFACT_DIR/covariance_worker_benchmark.json"
# Identical-horizon Ceres AutoDiff vs analytic A/B; runtime measurements
# are recorded but never hard-gated on shared CI runners.
/ws/build/mhe_sensor_fusion/test_ceres_analytic \
  --output "$ARTIFACT_DIR/ceres_analytic_ab.json"
echo '::endgroup::'

echo '::group::2. Deterministic and Monte Carlo synthetic tests'
"$PKG/benchmark/run_benchmarks.sh" 2>&1 | tee "$ARTIFACT_DIR/offline_benchmarks.log"
# Fast-path regression also exercises angular bypass and time-aligned yaw gates.
python3 "$PKG/ci/check_regressions.py" --package "$PKG" --output "$ARTIFACT_DIR/benchmark_summary.json"
(cd "$PKG/ci" && python3 -m unittest -v test_se2_cov_config)
python3 "$PKG/ci/test_ci_harness.py"
echo '::endgroup::'

echo '::group::3. Real ROS node / Ceres integration smoke test'
# No Gazebo required; publish timestamped synthetic IMU+wheel odometry and
# verify the ACTUAL mhe_sensor_fusion_node via ROS 2 DDS.
ros2 run mhe_sensor_fusion mhe_sensor_fusion_node --ros-args \
  --params-file /ws/install/mhe_sensor_fusion/share/mhe_sensor_fusion/config/mhe.yaml \
  -p use_sim_time:=false \
  -p publish_tf:=false \
  -p output_topic:=/mhe_ci/odom >"$ARTIFACT_DIR/mhe_node.log" 2>&1 &
NODE_PID=$!
cleanup_node() {
  kill "$NODE_PID" 2>/dev/null || true
  wait "$NODE_PID" 2>/dev/null || true
}
trap 'cleanup_node; archive_logs' EXIT
python3 "$PKG/ci/ros_smoke_test.py" --output "$ARTIFACT_DIR/ros_smoke.json"
python3 "$PKG/ci/check_rt_regressions.py" \
  --input "$ARTIFACT_DIR/ros_smoke.json" \
  --output "$ARTIFACT_DIR/rt_gate.json"
python3 "$PKG/ci/test_rt_gate.py"
python3 "$PKG/ci/compare_baseline.py" \
  --baseline "$PKG/ci/baseline_docker_ros_smoke.json" \
  --current "$ARTIFACT_DIR/ros_smoke.json" \
  --output "$ARTIFACT_DIR/rt_comparison.json"
# Test must fail if the node exited unexpectedly during the test.
kill -0 "$NODE_PID"
cleanup_node
trap archive_logs EXIT
echo '::endgroup::'

echo '::group::3b. Real ROS node full-rebuild vs incremental A/B'
"$PKG/ci/run_graph_ab.sh" "$ARTIFACT_DIR"
echo '::group::3c. Real ROS node legacy vs rank-aware marginal prior A/B'
"$PKG/ci/run_rank_prior_ab.sh" "$ARTIFACT_DIR"
echo '::endgroup::' 
echo '::endgroup::'

"$PKG/ci/run_yaw_so2_ab.sh" "$ARTIFACT_DIR"
"$PKG/ci/run_gyro_increment_ab.sh" "$ARTIFACT_DIR"
"$PKG/ci/run_se2_covariance_ab.sh" "$ARTIFACT_DIR"
echo '::group::4. Long-stress consistency unit tests and ROS stress regression'
(cd "$PKG/ci" && python3 -m unittest -v test_consistency_metrics test_long_stress_gate test_statistical_calibration test_stress_alignment test_yaw_metrics)
# A 25s fault-cycle is part of every CI. Set MHE_STRESS_SECONDS=600/1800
# for genuine long-duration testing; results use separate artifacts.
MHE_STRESS_SECONDS=${MHE_STRESS_SECONDS:-25} "$PKG/ci/run_long_stress.sh"
echo '::endgroup::'

echo 'CI OK: ROS build, unit tests, benchmarks, smoke and long-stress gate passed.'
