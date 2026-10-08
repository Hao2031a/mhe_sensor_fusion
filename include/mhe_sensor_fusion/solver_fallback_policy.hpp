#pragma once

#include <algorithm>
#include <cmath>

namespace mhe_fusion::safety
{

// Keep rejection reasons separate: trying QR again can help numerical/linear
// solver failures, but it cannot make a physically implausible endpoint valid.
enum class SolveRejectReason : int
{
  None = 0,
  Numerical = 1,
  Cost = 2,
  Endpoint = 3
};

inline SolveRejectReason classifySolveResult(
  bool usable, bool finite, bool acceptable_cost, bool acceptable_endpoint)
{
  if (!usable || !finite) {
    return SolveRejectReason::Numerical;
  }
  if (!acceptable_cost) {
    return SolveRejectReason::Cost;
  }
  if (!acceptable_endpoint) {
    return SolveRejectReason::Endpoint;
  }
  return SolveRejectReason::None;
}

struct QrFallbackPlan
{
  bool run{false};
  double remaining_ms{0.0};
};

// This is a *soft* compute budget: Ceres may overrun max_solver_time.
// Never start an independent second full-budget solve after Cholesky.
inline QrFallbackPlan planQrFallback(
  SolveRejectReason reason, bool first_solve_used_qr,
  double ceres_elapsed_ms, double solver_budget_ms,
  double min_remaining_ms, bool retry_endpoint_jump = false)
{
  if (reason == SolveRejectReason::None || first_solve_used_qr ||
      (reason == SolveRejectReason::Endpoint && !retry_endpoint_jump) ||
      !std::isfinite(ceres_elapsed_ms) || !std::isfinite(solver_budget_ms) ||
      !std::isfinite(min_remaining_ms) || ceres_elapsed_ms < 0.0 ||
      solver_budget_ms <= 0.0 || min_remaining_ms < 0.0) {
    return {};
  }
  const double remaining = std::max(0.0, solver_budget_ms - ceres_elapsed_ms);
  const double minimum = std::max(0.5, min_remaining_ms);
  if (remaining < minimum) {
    return {};
  }
  return {true, remaining};
}

}  // namespace mhe_fusion::safety
