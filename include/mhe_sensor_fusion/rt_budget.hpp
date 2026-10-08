#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace mhe_fusion::rt
{

// This is an ADVISORY scheduling gate, not a hard real-time guarantee.
// A full Ceres solve cannot be preempted; covariance uses a background worker.
struct CovarianceSchedule
{
  int every_n_solves{20};
  int max_age_solves{60};
  double budget_ms{8.0};
  double reserve_ms{2.5};
  bool defer_on_overrun{true};
};

enum class CovarianceDecision { NotDue, Compute, Defer };

inline CovarianceDecision covarianceDecision(
  const CovarianceSchedule & cfg,
  std::size_t solve_count, std::size_t age_solves,
  bool has_valid_covariance, double elapsed_ms, double estimated_covariance_ms)
{
  if (!has_valid_covariance ||
      static_cast<int>(age_solves) >= std::max(1, cfg.max_age_solves)) {
    return CovarianceDecision::Compute;
  }
  if ((solve_count % static_cast<std::size_t>(std::max(1, cfg.every_n_solves))) != 0) {
    return CovarianceDecision::NotDue;
  }
  // Unknown/non-finite estimates are treated as unsafe rather than granting
  // extra budget. A forced refresh after max_age_solves prevents starvation.
  const double predicted_ms = std::isfinite(estimated_covariance_ms) &&
    estimated_covariance_ms > 0.0 ? estimated_covariance_ms : cfg.reserve_ms;
  if (cfg.defer_on_overrun &&
      (!std::isfinite(elapsed_ms) ||
       elapsed_ms + std::max(predicted_ms, cfg.reserve_ms) > cfg.budget_ms)) {
    return CovarianceDecision::Defer;
  }
  return CovarianceDecision::Compute;
}

// Never force a Jacobian snapshot into an already overloaded output tick.
// Stale covariance must instead fall back to conservative twist uncertainty.
inline bool canSnapshot(double tick_elapsed_ms, double solve_elapsed_ms,
  double deadline_ms, double snapshot_budget_ms, double reserve_ms,
  double estimated_snapshot_ms)
{
  if (!std::isfinite(tick_elapsed_ms) || !std::isfinite(solve_elapsed_ms) ||
      !std::isfinite(deadline_ms) || !std::isfinite(snapshot_budget_ms) ||
      deadline_ms <= 0.0 || snapshot_budget_ms <= 0.0) {return false;}
  const double predicted = std::isfinite(estimated_snapshot_ms) &&
    estimated_snapshot_ms > 0.0 ? estimated_snapshot_ms : 1.0;
  const double reserve = std::isfinite(reserve_ms) && reserve_ms >= 0.0 ?
    reserve_ms : 2.5;
  return solve_elapsed_ms <= snapshot_budget_ms &&
    tick_elapsed_ms + std::max(0.75, predicted) + reserve < deadline_ms;
}

inline bool deadlineMiss(double elapsed_ms, double budget_ms)
{
  return std::isfinite(elapsed_ms) && std::isfinite(budget_ms) &&
         budget_ms > 0.0 && elapsed_ms > budget_ms;
}

}  // namespace mhe_fusion::rt
