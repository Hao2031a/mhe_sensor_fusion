#pragma once
#include <algorithm>
#include <cmath>

namespace mhe_fusion::safety {

enum class GateState : int { Normal = 0, Soft = 1, Hard = 2 };

struct GateResult {
  GateState state{GateState::Normal};
  double sigma_scale{1.0};
};

// Single-dimensional approximate pre-fit NIS (model + sensor variance).
// Conservative thresholds leave high-dynamics measurements mostly intact.
inline GateResult classifyInnovation(
  double nis, double soft_nis, double hard_nis, double max_sigma_scale)
{
  if (!std::isfinite(nis) || nis < 0.0) {
    return {GateState::Hard, 1.0};
  }
  if (nis >= hard_nis && hard_nis > soft_nis) {
    return {GateState::Hard, 1.0};
  }
  if (nis > soft_nis && hard_nis > soft_nis) {
    return {GateState::Soft, std::clamp(
      std::sqrt(nis / std::max(soft_nis, 1e-6)), 1.0,
      std::max(1.0, max_sigma_scale))};
  }
  return {GateState::Normal, 1.0};
}

inline bool costAcceptable(double initial_cost, double final_cost, double tolerance = 1e-4)
{
  return std::isfinite(initial_cost) && std::isfinite(final_cost) &&
         initial_cost >= 0.0 && final_cost >= 0.0 &&
         final_cost <= initial_cost * (1.0 + std::max(0.0, tolerance)) + 1e-9;
}

inline bool endpointJumpAcceptable(
  double v, double w, double reference_v, double reference_w, double seconds,
  double max_linear_accel, double max_angular_accel,
  double linear_margin, double angular_margin)
{
  if (!std::isfinite(v) || !std::isfinite(w) ||
      !std::isfinite(reference_v) || !std::isfinite(reference_w) ||
      !std::isfinite(seconds)) {
    return false;
  }
  const double dt = std::clamp(seconds, 0.0, 0.25);
  return std::abs(v - reference_v) <=
           std::max(0.0, max_linear_accel) * dt + std::max(0.0, linear_margin) &&
         std::abs(w - reference_w) <=
           std::max(0.0, max_angular_accel) * dt + std::max(0.0, angular_margin);
}

}  // namespace mhe_fusion::safety
