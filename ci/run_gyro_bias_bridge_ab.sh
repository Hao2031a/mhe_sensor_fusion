#!/usr/bin/env bash
# Real ROS 2/Ceres node, synthetic input; not Gazebo ground truth.
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
  local label="$1" enabled="$2"
  ros2 run mhe_sensor_fusion mhe_sensor_fusion_node --ros-args \
    --params-file /ws/install/mhe_sensor_fusion/share/mhe_sensor_fusion/config/mhe.yaml \
    -p use_sim_time:=false -p publish_tf:=false -p output_topic:=/mhe_ci/odom \
    -p solver.gyro_increment_factor_enabled:=true \
    -p solver.gyro_increment_bias_bridge_enabled:="$enabled" \
    >"$OUT/gyro_bias_bridge_${label}_node.log" 2>&1 &
  NODE_PID=$!
  MHE_CI_SMOKE_SECONDS=${MHE_CI_SMOKE_SECONDS:-7} \
    python3 "$PKG/ci/ros_smoke_test.py" --output "$OUT/gyro_bias_bridge_${label}.json"
  kill -0 "$NODE_PID"
  cleanup
  sleep 0.4
}
run_one baseline false
run_one bridge true
python3 "$PKG/ci/compare_graph_ab.py" \
  --baseline "$OUT/gyro_bias_bridge_baseline.json" \
  --optimized "$OUT/gyro_bias_bridge_bridge.json" \
  --output "$OUT/gyro_bias_bridge_node_ab.json"
