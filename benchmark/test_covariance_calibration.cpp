#include <mhe_sensor_fusion/covariance_calibration.hpp>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <stdexcept>
#include <cmath>
int main() {
  using mhe_sensor_fusion::calibratePublishedPoseCovariance;
  Eigen::Matrix3d p;
  p << 0.04, 0.01, 0.003,
       0.01, 0.09, 0.004,
       0.003, 0.004, 0.01;
  const auto same = calibratePublishedPoseCovariance(p, Eigen::Vector3d::Ones());
  if (!same.isApprox(p, 1e-12)) return 1;
  Eigen::Vector3d s(0.5, 1.5, 2.0);
  const auto q = calibratePublishedPoseCovariance(p, s);
  if (std::abs(q(0,0) - 0.01) > 1e-12 ||
      std::abs(q(0,1) - 0.0075) > 1e-12 ||
      std::abs(q(2,2) - 0.04) > 1e-12) return 2;
  if (Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(q).eigenvalues().minCoeff() <= 0) return 3;
  for (const auto & bad : {Eigen::Vector3d(0.0, 1.0, 1.0),
                            Eigen::Vector3d(2.1, 1.0, 1.0),
                            Eigen::Vector3d(NAN, 1.0, 1.0)}) {
    bool rejected = false;
    try { (void)calibratePublishedPoseCovariance(p, bad); }
    catch (const std::invalid_argument &) { rejected = true; }
    if (!rejected) return 4;
  }
  std::cout << "PASS: covariance calibration PSD, correlation and bound tests\n";
  return 0;
}
