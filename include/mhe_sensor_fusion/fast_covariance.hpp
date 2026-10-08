#pragma once

// Covariance worker independent of ROS / Ceres: callback hands off an immutable
// compressed-row Jacobian; the worker performs J^T J and selected-column solves.
// No shared reference to Ceres::Problem, state buffers, or ROS node is retained.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>
#include "mhe_sensor_fusion/scaled_covariance.hpp"

namespace mhe_fusion::fast_cov
{

struct Snapshot
{
  uint64_t generation{0};
  uint64_t sequence{0};
  int rows{0};
  int cols{0};
  int tail_size{9};
  bool jacobi_scaled_enabled{false};
  bool svd_fallback_enabled{true};
  double minimum_rcond{1e-10};
  double svd_relative_cutoff{1e-11};
  double maximum_svd_condition{1e10};
  std::vector<int> row_offsets;
  std::vector<int> column_indices;
  std::vector<double> values;
};

struct Result
{
  uint64_t generation{0};
  uint64_t sequence{0};
  int tail_size{0};
  bool valid{false};
  int method{0};  // 0=legacy, 1=scaled LLT, 2=SVD, 3=ill-conditioned
  int effective_rank{0};
  double reciprocal_condition{0.0};
  double compute_ms{0.0};
  // Row-major covariance of the final state. Invalid results have no matrix.
  std::vector<double> covariance;
};

// PSD normal-equation approximation. Damping is applied only to handle the
// last few machine precision bits of an otherwise identifiable window; strongly
// rank-deficient problems are rejected (conservative output fallback).
inline Result compute(const Snapshot & s)
{
  const auto begin = std::chrono::steady_clock::now();
  Result result;
  result.generation = s.generation;
  result.sequence = s.sequence;
  result.tail_size = s.tail_size;
  const int n = s.cols;
  const int k = s.tail_size;
  if (n <= 0 || n > 512 || k <= 0 || k > n || s.rows <= 0 ||
    s.row_offsets.size() != static_cast<size_t>(s.rows + 1) ||
    s.row_offsets.front() != 0 ||
    s.column_indices.size() != s.values.size() ||
    s.row_offsets.back() != static_cast<int>(s.values.size()))
  {
    return result;
  }

  if (s.jacobi_scaled_enabled) {
    // Factor Jacobian is immutable and lives only on this worker thread.
    Eigen::MatrixXd dense = Eigen::MatrixXd::Zero(s.rows, n);
    for (int row = 0; row < s.rows; ++row) {
      const int b = s.row_offsets[row], e = s.row_offsets[row + 1];
      if (b < 0 || e < b || e > static_cast<int>(s.values.size())) {return result;}
      for (int j = b; j < e; ++j) {
        const int col = s.column_indices[j];
        if (col < 0 || col >= n || !std::isfinite(s.values[j])) {return result;}
        dense(row, col) += s.values[j];
      }
    }
    scaled_cov::Config cfg;
    cfg.allow_svd = s.svd_fallback_enabled;
    cfg.minimum_rcond = s.minimum_rcond;
    cfg.svd_relative_cutoff = s.svd_relative_cutoff;
    cfg.maximum_svd_condition = s.maximum_svd_condition;
    const auto solved = scaled_cov::selected(dense, k, cfg);
    result.method = solved.method;
    result.effective_rank = solved.rank;
    result.reciprocal_condition = solved.reciprocal_condition;
    if (solved.valid) {
      result.covariance.resize(static_cast<size_t>(k*k));
      for (int i=0; i<k; ++i) {
        for (int j=0; j<k; ++j) {
          result.covariance[static_cast<size_t>(i*k+j)] = solved.covariance(i,j);
        }
      }
      result.valid = true;
    }
    result.compute_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - begin).count();
    return result;
  }

  std::vector<double> h(static_cast<size_t>(n * n), 0.0);
  for (int row = 0; row < s.rows; ++row) {
    const int b = s.row_offsets[row];
    const int e = s.row_offsets[row + 1];
    if (b < 0 || e < b || e > static_cast<int>(s.values.size())) {return result;}
    for (int a = b; a < e; ++a) {
      const int ca = s.column_indices[a];
      const double va = s.values[a];
      if (ca < 0 || ca >= n || !std::isfinite(va)) {return result;}
      for (int c = b; c <= a; ++c) {
        const int cc = s.column_indices[c];
        const double vc = s.values[c];
        if (cc < 0 || cc >= n || !std::isfinite(vc)) {return result;}
        h[static_cast<size_t>(std::max(ca, cc) * n + std::min(ca, cc))] += va * vc;
      }
    }
  }
  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      h[static_cast<size_t>(i * n + j)] = h[static_cast<size_t>(j * n + i)];
    }
  }

  // Scaling the ridge with the weakest diagonal is important: without this
  // bound, an unobservable slip state could acquire a falsely tiny variance.
  double min_diag = 1e300;
  for (int i = 0; i < n; ++i) {
    min_diag = std::min(min_diag, h[static_cast<size_t>(i * n + i)]);
  }
  if (!std::isfinite(min_diag) || min_diag < 1e-12) {return result;}
  const double ridge = std::max(1e-12, 1e-9 * min_diag);
  for (int i = 0; i < n; ++i) {
    h[static_cast<size_t>(i * n + i)] += ridge;
  }

  // Dense Cholesky of the full information matrix; only k RHS are solved.
  // For <=20 nine-state blocks this is bounded and avoids Ceres Dense SVD.
  std::vector<double> l(static_cast<size_t>(n * n), 0.0);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j <= i; ++j) {
      double sum = h[static_cast<size_t>(i * n + j)];
      for (int p = 0; p < j; ++p) {
        sum -= l[static_cast<size_t>(i * n + p)] *
          l[static_cast<size_t>(j * n + p)];
      }
      if (i == j) {
        if (!std::isfinite(sum) || sum < 1e-8 * min_diag) {return result;}
        l[static_cast<size_t>(i * n + j)] = std::sqrt(sum);
      } else {
        l[static_cast<size_t>(i * n + j)] =
          sum / l[static_cast<size_t>(j * n + j)];
      }
    }
  }

  result.covariance.assign(static_cast<size_t>(k * k), 0.0);
  std::vector<double> y(static_cast<size_t>(n));
  std::vector<double> x(static_cast<size_t>(n));
  for (int col = 0; col < k; ++col) {
    const int target = n - k + col;
    for (int i = 0; i < n; ++i) {
      double b = i == target ? 1.0 : 0.0;
      for (int j = 0; j < i; ++j) {
        b -= l[static_cast<size_t>(i * n + j)] * y[static_cast<size_t>(j)];
      }
      y[static_cast<size_t>(i)] = b / l[static_cast<size_t>(i * n + i)];
    }
    for (int i = n - 1; i >= 0; --i) {
      double b = y[static_cast<size_t>(i)];
      for (int j = i + 1; j < n; ++j) {
        b -= l[static_cast<size_t>(j * n + i)] * x[static_cast<size_t>(j)];
      }
      x[static_cast<size_t>(i)] = b / l[static_cast<size_t>(i * n + i)];
    }
    for (int row = 0; row < k; ++row) {
      result.covariance[static_cast<size_t>(row * k + col)] =
        x[static_cast<size_t>(n - k + row)];
    }
  }
  for (int row = 0; row < k; ++row) {
    for (int col = row + 1; col < k; ++col) {
      const auto a = static_cast<size_t>(row * k + col);
      const auto b = static_cast<size_t>(col * k + row);
      const double symmetric = 0.5 * (result.covariance[a] + result.covariance[b]);
      result.covariance[a] = result.covariance[b] = symmetric;
    }
    if (!std::isfinite(result.covariance[static_cast<size_t>(row * k + row)]) ||
      result.covariance[static_cast<size_t>(row * k + row)] <= 0.0) {return result;}
  }
  result.valid = true;
  result.compute_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - begin).count();
  return result;
}

// One pending job maximum, try-lock handoff and polling. Shutdown waits for
// worker without a deadline guarantee; never touched during a 100Hz callback.
class Worker
{
public:
  Worker() : thread_([this] {run();}) {}
  Worker(const Worker &) = delete;
  Worker & operator=(const Worker &) = delete;
  ~Worker()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopped_ = true;
    }
    cv_.notify_one();
    if (thread_.joinable()) {thread_.join();}
  }

  bool submit(Snapshot && value)
  {
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock() || stopped_ || pending_ || busy_) {return false;}
    pending_value_ = std::move(value);
    pending_ = true;
    cv_.notify_one();
    return true;
  }

  bool poll(Result & out)
  {
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock() || !result_ready_) {return false;}
    out = std::move(result_);
    result_ready_ = false;
    return true;
  }

private:
  void run()
  {
    while (true) {
      Snapshot s;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [&] {return stopped_ || pending_;});
        if (stopped_) {return;}
        s = std::move(pending_value_);
        pending_ = false;
        busy_ = true;
      }
      auto value = compute(s);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        result_ = std::move(value);
        result_ready_ = true;
        busy_ = false;
      }
    }
  }

  std::mutex mutex_;
  std::condition_variable cv_;
  std::thread thread_;
  bool stopped_{false};
  bool pending_{false};
  bool busy_{false};
  bool result_ready_{false};
  Snapshot pending_value_;
  Result result_;
};

}  // namespace mhe_fusion::fast_cov
