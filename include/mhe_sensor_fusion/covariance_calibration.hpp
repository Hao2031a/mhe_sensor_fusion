#pragma once
#include <Eigen/Core>
#include <cmath>
#include <stdexcept>

namespace mhe_sensor_fusion {
// D * P * D preserves positive semidefiniteness and pose correlations.
// Factors scale STANDARD DEVIATIONS (not variances). Defaults to identity.
inline Eigen::Matrix3d calibratePublishedPoseCovariance(
    const Eigen::Matrix3d & p, const Eigen::Vector3d & std_scale)
{
  for (int k = 0; k < 3; ++k) {
    if (!std::isfinite(std_scale[k]) || std_scale[k] < 0.5 || std_scale[k] > 2.0) {
      throw std::invalid_argument("pose_std_scales must be finite and in [0.5,2.0]");
    }
  }
  const Eigen::Matrix3d d = std_scale.asDiagonal();
  const Eigen::Matrix3d scaled = d * p * d;
  return 0.5 * (scaled + scaled.transpose());
}
} // namespace mhe_sensor_fusion
