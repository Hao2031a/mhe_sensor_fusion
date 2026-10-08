#pragma once

#include <algorithm>
#include <cmath>

namespace mhe_sensor_fusion
{

constexpr double kPi = 3.14159265358979323846;

inline double wrapYaw(double yaw) noexcept
{
  return std::remainder(yaw, 2.0 * kPi);
}

// Optional reference anchoring is deliberately independent of the ROS runtime
// so its timestamp, outlier and rate-limit invariants can be regression-tested.
struct YawFeedbackPolicy
{
  bool enabled{false};
  double gain{0.15};
  double max_rate_rad_s{0.20};
  double max_error_rad{0.20};
  double max_sensor_age_sec{0.04};
  // Trust gates protect against warm-start only endpoints and degraded sensors.
  double max_reference_skew_sec{0.002};
  double min_confidence{0.35};
  double max_gyro_prefit_nis{16.0};
  bool require_covariance{false};
  double max_yaw_variance{0.10};
  unsigned int max_covariance_age_solves{80};
};

struct YawFeedbackResult
{
  bool applied{false};
  double error_rad{0.0};
  double step_rad{0.0};
};

inline bool trustworthyYawEndpoint(
  bool accepted_this_endpoint, double reference_skew_sec,
  double confidence, double gyro_prefit_nis,
  bool covariance_valid, double yaw_variance,
  unsigned int covariance_age_solves,
  const YawFeedbackPolicy & policy) noexcept
{
  if (!accepted_this_endpoint || !std::isfinite(reference_skew_sec) ||
      std::abs(reference_skew_sec) > policy.max_reference_skew_sec ||
      !std::isfinite(confidence) || confidence < policy.min_confidence ||
      !std::isfinite(gyro_prefit_nis) ||
      gyro_prefit_nis > policy.max_gyro_prefit_nis) {
    return false;
  }
  if (policy.require_covariance &&
      (!covariance_valid || !std::isfinite(yaw_variance) ||
       yaw_variance < 0.0 || yaw_variance > policy.max_yaw_variance ||
       covariance_age_solves > policy.max_covariance_age_solves)) {
    return false;
  }
  return true;
}

inline YawFeedbackResult boundedYawFeedback(
  double published_yaw, double endpoint_yaw, double endpoint_angular_rate,
  double sensor_age_sec, double output_dt_sec, const YawFeedbackPolicy & policy) noexcept
{
  YawFeedbackResult result;
  if (!policy.enabled || !std::isfinite(published_yaw) ||
      !std::isfinite(endpoint_yaw) || !std::isfinite(endpoint_angular_rate) ||
      !std::isfinite(sensor_age_sec) || !std::isfinite(output_dt_sec) ||
      sensor_age_sec < 0.0 || sensor_age_sec > policy.max_sensor_age_sec ||
      output_dt_sec <= 0.0 || output_dt_sec > 0.1 ||
      policy.gain <= 0.0 || !std::isfinite(policy.gain) ||
      policy.max_rate_rad_s <= 0.0 || !std::isfinite(policy.max_rate_rad_s) ||
      policy.max_error_rad <= 0.0 || !std::isfinite(policy.max_error_rad)) {
    return result;
  }

  // Compare poses at the SAME simulation timestamp. The Ceres endpoint pose
  // is not a pose at the current publisher tick.
  const double reference_yaw = wrapYaw(endpoint_yaw + endpoint_angular_rate * sensor_age_sec);
  const double error = wrapYaw(reference_yaw - published_yaw);
  if (!std::isfinite(error) || std::abs(error) > policy.max_error_rad) {
    return result;
  }
  result.error_rad = error;
  const double gain = std::clamp(policy.gain, 0.0, 1.0);
  const double max_step = policy.max_rate_rad_s * output_dt_sec;
  result.step_rad = std::clamp(gain * error, -max_step, max_step);
  result.applied = std::abs(result.step_rad) > 1e-12;
  return result;
}

}  // namespace mhe_sensor_fusion
