#!/usr/bin/env bash
# Two actual ROS 2 Ceres-node smoke runs, same scripted motions. The wall-clock
# timing comparison is diagnostic only, NOT a deterministic performance gate.
set -Eeuo pipefail
set +u
source /opt/ros/${ROS_DISTRO:-kilted}/setup.bash
source /ws/install/setup.bash
set -u
PKG=/ws/src/mhe_sensor_fusion
OUT=${1:-${CI_ARTIFACT_DIR:-/artifacts}}
mkdir -p "$OUT"
NODE_PID=
cleanup() {
  if [[ -n "$NODE_PID" ]]; then
    kill "$NODE_PID" 2>/dev/null || true
    wait "$NODE_PID" 2>/dev/null || true
    NODE_PID=
  fi
}
trap cleanup EXIT
run_one() {
  local label="$1" incremental="$2" schur="$3"
  ros2 run mhe_sensor_fusion mhe_sensor_fusion_node --ros-args \
    --params-file /ws/install/mhe_sensor_fusion/share/mhe_sensor_fusion/config/mhe.yaml \
    -p use_sim_time:=false -p publish_tf:=false -p output_topic:=/mhe_ci/odom \
    -p solver.incremental_graph_enabled:="$incremental" \
    -p solver.block_schur_enabled:="$schur" \
    >"$OUT/graph_ab_${label}_node.log" 2>&1 &
  NODE_PID=$!
  MHE_CI_SMOKE_SECONDS=${MHE_CI_SMOKE_SECONDS:-7} \
    python3 "$PKG/ci/ros_smoke_test.py" --output "$OUT/graph_ab_${label}.json"
  kill -0 "$NODE_PID"
  cleanup
  sleep 0.4
}
run_one full_rebuild false false
run_one incremental true true
python3 "$PKG/ci/compare_graph_ab.py" \
  --baseline "$OUT/graph_ab_full_rebuild.json" \
  --optimized "$OUT/graph_ab_incremental.json" \
  --output "$OUT/graph_ab_comparison.json"
