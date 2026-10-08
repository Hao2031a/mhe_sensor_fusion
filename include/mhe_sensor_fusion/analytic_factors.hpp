#pragma once

// Dependency-free residual and analytic Jacobian kernels for the hot MHE
// process/wheel factors. Matrix Jacobians use Ceres' row-major layout.
// The formulas must match the existing AutoDiff reference exactly; independent
// finite-difference and reference-residual tests guard this invariant.
#include <array>
#include <cmath>
#include <algorithm>
#include <cstddef>
#include "mhe_sensor_fusion/se2_yaw_math.hpp"

namespace mhe_fusion::analytic {
constexpr int kDimension = 9;
enum Index : int { kX = 0, kY, kYaw, kV, kW, kBg, kBa, kSl, kSr };

inline void clearJac(double * matrix, int rows)
{
  if (matrix) {
    std::fill(matrix, matrix + rows * kDimension, 0.0);
  }
}

struct ProcessKernel
{
  explicit ProcessKernel(double delta_t, const std::array<double, kDimension> & sigma,
    bool exact_arc = false, bool so2_yaw = false)
  : dt(delta_t), exact_arc_(exact_arc), so2_yaw_(so2_yaw)
  {
    for (int i = 0; i < kDimension; ++i) {
      inv_sigma[i] = 1.0 / sigma[i];
    }
  }

  bool evaluate(const double * x0, const double * x1, double * residual,
                double * jac0, double * jac1) const
  {
    if (!x0 || !x1 || !residual || !(dt > 0.0)) {return false;}
    const double vbar = 0.5 * (x0[kV] + x1[kV]);
    const double wbar = 0.5 * (x0[kW] + x1[kW]);
    const double mid = x0[kYaw] + 0.5 * dt * wbar;
    const double cosine = std::cos(mid);
    const double sine = std::sin(mid);
    const double move = dt * vbar;
    yaw_math::ArcDerivatives arc;
    if (exact_arc_) {
      arc = yaw_math::arcWithDerivatives(x0[kYaw], vbar, wbar, dt);
    }
    residual[kX] = (x1[kX] - x0[kX] -
      (exact_arc_ ? arc.dx : move * cosine)) * inv_sigma[kX];
    residual[kY] = (x1[kY] - x0[kY] -
      (exact_arc_ ? arc.dy : move * sine)) * inv_sigma[kY];
    const double angle_diff = x1[kYaw] - x0[kYaw] - dt * wbar;
    residual[kYaw] = (so2_yaw_ ? yaw_math::wrapDifference(angle_diff) :
      angle_diff) * inv_sigma[kYaw];
    for (int i = kV; i < kDimension; ++i) {
      residual[i] = (x1[i] - x0[i]) * inv_sigma[i];
    }
    if (!jac0 && !jac1) {return true;}

    // x/y depend on both velocity endpoints and the midpoint yaw.
    // yaw_mid = yaw0 + (dt / 4) * (w0 + w1).
    const double dx_dyaw = (exact_arc_ ? -arc.dx_dyaw : move * sine) * inv_sigma[kX];
    const double dy_dyaw = (exact_arc_ ? -arc.dy_dyaw : -move * cosine) * inv_sigma[kY];
    const double dx_dv = (exact_arc_ ? -0.5 * arc.dx_dv : -0.5 * dt * cosine) * inv_sigma[kX];
    const double dy_dv = (exact_arc_ ? -0.5 * arc.dy_dv : -0.5 * dt * sine) * inv_sigma[kY];
    const double dx_dw = (exact_arc_ ? -0.5 * arc.dx_dw * inv_sigma[kX] :
      dx_dyaw * 0.25 * dt);
    const double dy_dw = (exact_arc_ ? -0.5 * arc.dy_dw * inv_sigma[kY] :
      dy_dyaw * 0.25 * dt);
    const double dyaw_dw = -0.5 * dt * inv_sigma[kYaw];

    if (jac0) {
      clearJac(jac0, kDimension);
      for (int i = 0; i < kDimension; ++i) {
        jac0[i * kDimension + i] = -inv_sigma[i];
      }
      jac0[kX * kDimension + kYaw] = dx_dyaw;
      jac0[kY * kDimension + kYaw] = dy_dyaw;
      jac0[kX * kDimension + kV] = dx_dv;
      jac0[kY * kDimension + kV] = dy_dv;
      jac0[kX * kDimension + kW] = dx_dw;
      jac0[kY * kDimension + kW] = dy_dw;
      jac0[kYaw * kDimension + kW] = dyaw_dw;
    }
    if (jac1) {
      clearJac(jac1, kDimension);
      for (int i = 0; i < kDimension; ++i) {
        jac1[i * kDimension + i] = inv_sigma[i];
      }
      jac1[kX * kDimension + kV] = dx_dv;
      jac1[kY * kDimension + kV] = dy_dv;
      jac1[kX * kDimension + kW] = dx_dw;
      jac1[kY * kDimension + kW] = dy_dw;
      jac1[kYaw * kDimension + kW] = dyaw_dw;
    }
    return true;
  }

  double dt;
  bool exact_arc_{false};
  bool so2_yaw_{false};
  std::array<double, kDimension> inv_sigma{};
};

struct WheelKernel
{
  WheelKernel(double left, double right, double separation,
              double sigma_left, double sigma_right)
  : wheel_left(left), wheel_right(right), half_track(0.5 * separation),
    inv_sigma_left(1.0 / sigma_left), inv_sigma_right(1.0 / sigma_right) {}

  bool evaluate(const double * x, double * residual, double * jac) const
  {
    if (!x || !residual) {return false;}
    const double vl = x[kV] - half_track * x[kW];
    const double vr = x[kV] + half_track * x[kW];
    const double ls = 1.0 + x[kSl];
    const double rs = 1.0 + x[kSr];
    residual[0] = (ls * vl - wheel_left) * inv_sigma_left;
    residual[1] = (rs * vr - wheel_right) * inv_sigma_right;
    if (!jac) {return true;}
    clearJac(jac, 2);
    jac[kV] = ls * inv_sigma_left;
    jac[kW] = -half_track * ls * inv_sigma_left;
    jac[kSl] = vl * inv_sigma_left;
    jac[kDimension + kV] = rs * inv_sigma_right;
    jac[kDimension + kW] = half_track * rs * inv_sigma_right;
    jac[kDimension + kSr] = vr * inv_sigma_right;
    return true;
  }

  double wheel_left, wheel_right, half_track, inv_sigma_left, inv_sigma_right;
};
}  // namespace mhe_fusion::analytic
