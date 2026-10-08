#!/usr/bin/env bash
# Actual ROS 2 node + Ceres, synthetic timestamped sensor inputs, NOT Gazebo.
set -Eeuo pipefail
set +u
source /opt/ros/${ROS_DISTRO:-kilted}/setup.bash
source /ws/install/setup.bash
set -u
PKG=/ws/src/mhe_sensor_fusion
OUT=${1:-${CI_ARTIFACT_DIR:-/artifacts}}
mkdir -p "$OUT"
NODE_PID=
cleanup(){ if [[ -n "$NODE_PID" ]]; then kill "$NODE_PID" 2>/dev/null || true; wait "$NODE_PID" 2>/dev/null || true; NODE_PID=; fi; }
trap cleanup EXIT
run_one(){
  local label="$1" corrected="$2"
  ros2 run mhe_sensor_fusion mhe_sensor_fusion_node --ros-args \
    --params-file /ws/install/mhe_sensor_fusion/share/mhe_sensor_fusion/config/mhe.yaml \
    -p use_sim_time:=false -p publish_tf:=false -p output_topic:=/mhe_ci/odom \
    -p output.smoothing.angular_enabled:=false \
    -p output.yaw_feedback.enabled:=false \
    -p covariance.jacobi_scaled_enabled:="$corrected" \
    -p covariance.svd_fallback_enabled:=true \
    -p output.se2_correction.enabled:="$corrected" \
    >"$OUT/math_${label}_node.log" 2>&1 &
  NODE_PID=$!
  MHE_CI_SMOKE_SECONDS=${MHE_CI_SMOKE_SECONDS:-8} \
    python3 "$PKG/ci/ros_smoke_test.py" --output "$OUT/math_${label}.json"
  kill -0 "$NODE_PID"
  cleanup
  sleep 0.4
}
run_one legacy false
run_one scaled_se2 true
python3 "$PKG/ci/compare_graph_ab.py" \
  --baseline "$OUT/math_legacy.json" \
  --optimized "$OUT/math_scaled_se2.json" \
  --output "$OUT/math_node_ab.json"
