#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
cd "${PACKAGE_DIR}"

echo "[1/8] Core upgrade benchmark"
python3 benchmark/benchmark_upgrade.py

echo "[2/8] Deterministic output-stability benchmark"
python3 benchmark/benchmark_stability.py

echo "[3/8] 200-trial/channel Monte-Carlo stability benchmark"
python3 benchmark/benchmark_stability_mc.py

echo "[4/8] C++ guards compiled unit tests"
GUARD_TEST_BIN="$(mktemp /tmp/mhe_test_stability_guards.XXXXXX)"
trap 'rm -f "$GUARD_TEST_BIN"' EXIT
g++ -std=c++17 -O2 -Wall -Wextra -Werror -I"${PACKAGE_DIR}/include" \
  "${SCRIPT_DIR}/test_stability_guards.cpp" -o "$GUARD_TEST_BIN"
"$GUARD_TEST_BIN"

echo "[5/8] Real-time covariance scheduling unit tests"
RT_TEST_BIN="$(mktemp /tmp/mhe_test_rt_budget.XXXXXX)"
trap 'rm -f "$GUARD_TEST_BIN" "$RT_TEST_BIN"' EXIT
g++ -std=c++17 -O2 -Wall -Wextra -Werror -I"${PACKAGE_DIR}/include" \
  "${SCRIPT_DIR}/test_rt_budget.cpp" -o "$RT_TEST_BIN"
"$RT_TEST_BIN"

echo "[6/8] 200k-event innovation gating / pose-covariance Monte Carlo"
python3 benchmark/benchmark_innovation_gate.py

echo "[7/8] Source integration invariants"
python3 benchmark/test_source_invariants.py

echo "[8/8] Independent Gaussian NEES/NIS calibration regression"
python3 benchmark/benchmark_consistency_mc.py

echo "[9/10] Low-latency yaw output regression (30 deterministic synthetic seeds)"
python3 benchmark/benchmark_yaw_latency.py

echo "[10/10] C++ yaw correction timestamp/rate-limit unit tests"
YAW_TEST_BIN="$(mktemp /tmp/mhe_test_low_latency_output.XXXXXX)"
trap 'rm -f "$GUARD_TEST_BIN" "$RT_TEST_BIN" "$YAW_TEST_BIN"' EXIT
g++ -std=c++17 -O2 -Wall -Wextra -Werror -I"${PACKAGE_DIR}/include" \
  "${SCRIPT_DIR}/test_low_latency_output.cpp" -o "$YAW_TEST_BIN"
"$YAW_TEST_BIN"

echo "[11/11] Synthetic QR fallback policy benchmark (30 seeds)"
python3 benchmark/benchmark_solver_fallback.py

echo "[12/13] C++ time alignment and yaw trust regression"
TIME_TEST_BIN="$(mktemp /tmp/mhe_test_time_aligned_output.XXXXXX)"
trap 'rm -f "$GUARD_TEST_BIN" "$RT_TEST_BIN" "$YAW_TEST_BIN" "$TIME_TEST_BIN"' EXIT
g++ -std=c++17 -O2 -DNDEBUG -Wall -Wextra -Werror -I"${PACKAGE_DIR}/include" \
  "${SCRIPT_DIR}/test_time_aligned_output.cpp" -o "$TIME_TEST_BIN"
"$TIME_TEST_BIN"

echo "[13/13] Timestamp-aligned yaw Monte Carlo regression"
python3 benchmark/benchmark_timestamp_yaw.py

echo "[14/14] Analytic factor residual/Jacobian finite-difference regression"
ANALYTIC_TEST_BIN="$(mktemp /tmp/mhe_test_analytic_factors.XXXXXX)"
trap 'rm -f "$GUARD_TEST_BIN" "$RT_TEST_BIN" "$YAW_TEST_BIN" "$TIME_TEST_BIN" "$ANALYTIC_TEST_BIN"' EXIT
g++ -std=c++17 -O2 -DNDEBUG -Wall -Wextra -Werror -I"${PACKAGE_DIR}/include" \
  "${SCRIPT_DIR}/test_analytic_factors.cpp" -o "$ANALYTIC_TEST_BIN"
"$ANALYTIC_TEST_BIN"
echo "[incremental] Synthetic chain Schur numerical reference regression"
python3 benchmark/benchmark_block_schur.py

echo "[incremental] Exact-set factor-selection test (all Release/Debug builds)"
SELECTOR_TEST_BIN="$(mktemp /tmp/mhe_test_factor_selector.XXXXXX)"
trap 'rm -f "$GUARD_TEST_BIN" "$RT_TEST_BIN" "$YAW_TEST_BIN" "$TIME_TEST_BIN" "$ANALYTIC_TEST_BIN" "$SELECTOR_TEST_BIN"' EXIT
g++ -std=c++17 -O2 -DNDEBUG -Wall -Wextra -Werror -I"${PACKAGE_DIR}/include" \
  "${SCRIPT_DIR}/test_marginal_factor_selector.cpp" -o "$SELECTOR_TEST_BIN"
"$SELECTOR_TEST_BIN"

echo "[rank-aware] Synthetic rank-preserving marginal prior benchmark"
python3 benchmark/benchmark_rank_aware_prior.py

echo "[SO2 yaw] Analytic arc/Jacobian and angle-wrap C++ regression"
SO2_TEST_BIN="$(mktemp /tmp/mhe_test_so2_yaw.XXXXXX)"
g++ -std=c++17 -O2 -Wall -Wextra -Werror -I"${PACKAGE_DIR}/include" \
  "${SCRIPT_DIR}/test_so2_yaw_math.cpp" -o "$SO2_TEST_BIN"
"$SO2_TEST_BIN"
rm -f "$SO2_TEST_BIN"

echo "[SO2 yaw] Independent quadrature and +/-pi regression"
python3 benchmark/benchmark_so2_yaw.py

echo "ALL BENCHMARKS PASSED (including SO2 yaw + exact SE2 arc)"

# Analytic gyro SO(2) delta factor and synthetic variance-envelope validation.
echo "[gyro increment] C++ Jacobian/closure regression"
GYRO_TEST_BIN="$(mktemp /tmp/mhe_test_gyro_increment.XXXXXX)"
trap 'rm -f "$GYRO_TEST_BIN"' EXIT
g++ -std=c++17 -O2 -DNDEBUG -Wall -Wextra -Werror -I"${PACKAGE_DIR}/include" \
  "${SCRIPT_DIR}/test_gyro_increment.cpp" -o "$GYRO_TEST_BIN"
"$GYRO_TEST_BIN"
python3 benchmark/benchmark_gyro_increment.py
