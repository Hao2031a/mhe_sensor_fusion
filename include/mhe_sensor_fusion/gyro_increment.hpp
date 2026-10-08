#pragma once
#include <algorithm>
#include <cmath>
#include <limits>

namespace mhe_sensor_fusion::gyro_increment {
constexpr int NX = 9;
constexpr int YAW = 2;
constexpr int BG = 5;

// A single accepted gyro sample is interpreted as a ZOH rate averaged over
// the preceding short interval.  Rate measurement noise and quadrature-model
// uncertainty are independent, in units rad/s.  No sqrt(dt) shortcut: these
// are per-sample discrete rate uncertainties, not continuous-time PSDs.
inline double sigmaAngle(double dt, double sigma_rate, double sigma_model_rate) {
  return dt * std::hypot(sigma_rate, sigma_model_rate);
}

inline double wrap(double theta) {
  if (theta >= -3.14159265358979323846 && theta <= 3.14159265358979323846)
    return theta;
  return std::atan2(std::sin(theta), std::cos(theta));
}

class Kernel {
public:
  Kernel(double gyro_rate, double dt, double sigma_rate, double sigma_model_rate)
    : rate_(gyro_rate), dt_(dt), sigma_(sigmaAngle(dt, sigma_rate, sigma_model_rate)) {}

  // Ceres expects one row-major 1x9 Jacobian for each state block.
  // This factor has no w term: it measures yaw increment directly from the
  // integrated gyro rate, while the process factor already relates yaw and w.
  bool evaluate(const double* a, const double* b, double* r,
                double* ja = nullptr, double* jb = nullptr) const {
    if (!(std::isfinite(rate_) && std::isfinite(dt_) &&
          std::isfinite(sigma_) && dt_ > 0.0 && sigma_ > 0.0)) return false;
    const double residual = wrap(
      b[YAW] - a[YAW] - dt_ * (rate_ - 0.5 * (a[BG] + b[BG])));
    r[0] = residual / sigma_;
    if (ja) {
      std::fill_n(ja, NX, 0.0);
      ja[YAW] = -1.0 / sigma_;
      ja[BG] = 0.5 * dt_ / sigma_;
    }
    if (jb) {
      std::fill_n(jb, NX, 0.0);
      jb[YAW] = 1.0 / sigma_;
      jb[BG] = 0.5 * dt_ / sigma_;
    }
    return true;
  }
private:
  double rate_, dt_, sigma_;
};
}  // namespace mhe_sensor_fusion::gyro_increment
