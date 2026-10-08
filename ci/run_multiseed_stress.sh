#!/usr/bin/env bash
# Explicit multi-seed benchmark; each seed executes the REAL ROS/Ceres node.
set -Eeuo pipefail
PKG=/ws/src/mhe_sensor_fusion
BASE=${CI_ARTIFACT_DIR:-/artifacts}
CSV=${MHE_STRESS_SEEDS:-20261008,20261009,20261010}
IFS=',' read -ra SEEDS <<< "$CSV"
if (( ${#SEEDS[@]} < 3 )); then echo 'Require >=3 independent seeds' >&2; exit 2; fi
mkdir -p "$BASE"
FILES=()
for seed in "${SEEDS[@]}"; do
  if ! [[ $seed =~ ^[0-9]+$ ]]; then echo "Invalid seed: $seed" >&2; exit 2; fi
  echo "==== Seed $seed ===="
  MHE_STRESS_SEED="$seed" CI_ARTIFACT_DIR="$BASE/seed-$seed" \
    MHE_STRESS_CONSISTENCY_GATE=0 /bin/bash "$PKG/ci/run_long_stress.sh"
  FILES+=("$BASE/seed-$seed/long_stress.json")
done
# Two training runs (or more) + ONE reserved independent holdout.
python3 "$PKG/ci/propose_covariance_calibration.py" \
  --train "${FILES[@]:0:${#FILES[@]}-1}" \
  --holdout "${FILES[-1]}" \
  --output "$BASE/covariance_calibration_proposal.json"
if [[ ${MHE_STRESS_CONSISTENCY_GATE:-0} == 1 ]]; then
  for seed in "${SEEDS[@]}"; do
    MHE_STRESS_CONSISTENCY_GATE=1 \
      python3 "$PKG/ci/check_long_stress.py" \
      --input "$BASE/seed-$seed/long_stress.json" \
      --output "$BASE/seed-$seed/consistency_gate.json"
  done
fi
