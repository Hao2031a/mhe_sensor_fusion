#!/usr/bin/env bash
# A/B the actual ROS 2 Ceres node with identical scripted synthetic wheel+IMU
# scenarios. This is not a Gazebo / physical-ground-truth performance proof.
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
  local label="$1" rank_aware="$2"
  ros2 run mhe_sensor_fusion mhe_sensor_fusion_node --ros-args \
    --params-file /ws/install/mhe_sensor_fusion/share/mhe_sensor_fusion/config/mhe.yaml \
    -p use_sim_time:=false -p publish_tf:=false -p output_topic:=/mhe_ci/odom \
    -p solver.incremental_graph_enabled:=true \
    -p solver.block_schur_enabled:=true \
    -p solver.rank_aware_prior_enabled:="$rank_aware" \
    >"$OUT/rank_prior_${label}_node.log" 2>&1 &
  NODE_PID=$!
  MHE_CI_SMOKE_SECONDS=${MHE_CI_SMOKE_SECONDS:-7} \
    python3 "$PKG/ci/ros_smoke_test.py" --output "$OUT/rank_prior_${label}.json"
  kill -0 "$NODE_PID"
  cleanup
  sleep 0.4
}
run_one legacy false
run_one rank_aware true
python3 "$PKG/ci/compare_graph_ab.py" \
  --baseline "$OUT/rank_prior_legacy.json" \
  --optimized "$OUT/rank_prior_rank_aware.json" \
  --output "$OUT/rank_prior_ab_comparison.json"
