#include "mhe_sensor_fusion/fast_covariance.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <iostream>
#include <thread>
using namespace mhe_fusion::fast_cov;

static void check(bool good, const char * label)
{
  if (!good) {std::cerr << "FAIL: " << label << '\n'; std::exit(1);}
}

int main(int argc, char ** argv)
{
  int checks = 0;
  // J = [[2, 0], [0, 4]] -> P = diag(0.25, 0.0625).
  Snapshot s;
  s.rows = 2; s.cols = 2; s.tail_size = 2;
  s.row_offsets = {0, 1, 2}; s.column_indices = {0, 1};
  s.values = {2.0, 4.0}; s.generation = 3; s.sequence = 42;
  const auto r = compute(s);
  check(r.valid && r.generation == 3 && r.sequence == 42, "valid typed snapshot"); ++checks;
  check(std::abs(r.covariance[0] - .25) < 1e-7, "variance col 0"); ++checks;
  check(std::abs(r.covariance[3] - .0625) < 1e-7, "variance col 1"); ++checks;
  check(std::abs(r.covariance[1]) < 1e-9, "off diagonal 0"); ++checks;

  // Only extract marginal of last parameter: full (JtJ)^-1, not 1/H22.
  s.rows = 2; s.row_offsets = {0, 2, 3};
  s.column_indices = {0, 1, 1}; s.values = {1.0, 1.0, 1.0}; s.tail_size = 1;
  const auto correlated = compute(s);
  check(correlated.valid && std::abs(correlated.covariance[0] - 1.0) < 1e-6,
    "cross-correlated marginal is Schur complement"); ++checks;
  s.column_indices = {0, 0, 0};
  check(!compute(s).valid, "reject unidentifiable parameter"); ++checks;
  // Nonzero diagonal is not proof of observability: J=[1 1] is rank-1.
  Snapshot rank_def;
  rank_def.rows = 1; rank_def.cols = 2; rank_def.tail_size = 1;
  rank_def.row_offsets = {0, 2}; rank_def.column_indices = {0, 1};
  rank_def.values = {1.0, 1.0};
  check(!compute(rank_def).valid, "reject coupled but rank-deficient information"); ++checks;
  s.row_offsets = {0, 5, 3};
  check(!compute(s).valid, "reject malformed CSR"); ++checks;

  s = Snapshot{};
  s.rows = 2; s.cols = 2; s.tail_size = 2;
  s.row_offsets = {0, 1, 2}; s.column_indices = {0, 1};
  s.values = {2.0, 4.0}; s.sequence = 99; s.generation = 7;
  {
    Worker worker;
    check(worker.submit(std::move(s)), "submit immutable snapshot"); ++checks;
    Result got;
    bool ready = false;
    for (int i = 0; i < 500 && !ready; ++i) {
      ready = worker.poll(got);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    check(ready && got.valid && got.sequence == 99 && got.generation == 7,
      "receive worker result"); ++checks;
    check(!worker.poll(got), "consume each result at most once"); ++checks;
  }

  // 13x9-state window with banded motion factors. Benchmarks the actual
  // background normal-equation calculation, not a constant-time surrogate.
  constexpr int n = 13 * 9;
  Snapshot window;
  window.rows = 2 * n - 1;
  window.cols = n;
  window.tail_size = 9;
  window.row_offsets.reserve(2 * n);
  window.row_offsets.push_back(0);
  for (int i = 0; i < n; ++i) {
    window.column_indices.push_back(i);
    window.values.push_back(2.0 + 0.01 * static_cast<double>(i % 9));
    window.row_offsets.push_back(static_cast<int>(window.values.size()));
  }
  for (int i = 1; i < n; ++i) {
    window.column_indices.push_back(i - 1);
    window.column_indices.push_back(i);
    window.values.push_back(-1.0);
    window.values.push_back(1.0);
    window.row_offsets.push_back(static_cast<int>(window.values.size()));
  }
  std::vector<double> measurements;
  constexpr int trials = 100;
  for (int i = 0; i < trials; ++i) {
    const auto started = std::chrono::steady_clock::now();
    auto result = compute(window);
    const auto elapsed = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    check(result.valid, "full 117-state-column snapshot solvable");
    check(result.covariance[0] > 0.0 && result.covariance[0] < 1.0,
      "full snapshot covariance bounded");
    measurements.push_back(elapsed);
  }
  ++checks; ++checks;
  std::sort(measurements.begin(), measurements.end());
  const double p50 = measurements[50], p95 = measurements[95], p99 = measurements[99];
  std::cout << "Synthetic 117-column fast covariance P50=" << p50
            << "ms P95=" << p95 << "ms P99=" << p99 << "ms\n";
  if (argc == 3 && std::string(argv[1]) == "--benchmark-output") {
    std::ofstream out(argv[2]);
    check(static_cast<bool>(out), "write worker benchmark JSON");
    out << "{\"pass\":true,\"kind\":\"offline_synthetic\",\"columns\":117,"
        << "\"trials\":100,\"p50_ms\":" << p50
        << ",\"p95_ms\":" << p95 << ",\"p99_ms\":" << p99 << "}\n";
  }
  std::cout << "PASS: " << checks << " asynchronous fast covariance tests\n";
}
