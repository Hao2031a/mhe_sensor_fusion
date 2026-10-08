#include "mhe_sensor_fusion/rank_aware_prior.hpp"
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <random>
#include <cmath>

using Matrix = Eigen::Matrix<double, 9, 9>;
using Vector = Eigen::Matrix<double, 9, 1>;

int main()
{
  std::mt19937_64 rng(20261008);
  std::normal_distribution<double> gaussian;
  int checks = 0;
  double max_information_relative_error = 0.0;
  double max_gradient_relative_error = 0.0;
  for (int rank = 1; rank <= 9; ++rank) {
    for (int run = 0; run < 150; ++run) {
      Matrix raw;
      for (int i = 0; i < 9; ++i) for (int j = 0; j < 9; ++j) raw(i,j) = gaussian(rng);
      Eigen::HouseholderQR<Matrix> qr(raw);
      const Matrix Q = qr.householderQ();
      Vector lambdas = Vector::Zero();
      for (int i = 0; i < rank; ++i) lambdas[i] = 0.3 + static_cast<double>(i) * 0.2;
      // Mixed physical dimensions: units for pose, yaw, v, omega, bias, slip.
      Vector diag;
      diag << 1.0, 2.0, 1.3, 4.0, 0.8, 0.2, 0.1, 0.7, 1.4;
      Matrix H = diag.asDiagonal() * Q * lambdas.asDiagonal() * Q.transpose() * diag.asDiagonal();
      Vector delta;
      for (int i = 0; i < 9; ++i) delta[i] = gaussian(rng);
      Vector g = H * delta; // A physically consistent gradient in range(H).
      auto prior = mhe_sensor_fusion::rank_aware::factorize<9>(H, g, 1e-10);
      if (!prior.valid || prior.rank != rank) {
        std::cerr << "rank mismatch " << rank << " vs " << prior.rank
                  << " valid=" << prior.valid << "\n";
        return 1;
      }
      const double errH = (H - prior.sqrt_info.transpose() * prior.sqrt_info).norm() /
        std::max(1.0, H.norm());
      const double errG = (g - prior.sqrt_info.transpose() * prior.offset).norm() /
        std::max(1.0, g.norm());
      max_information_relative_error = std::max(max_information_relative_error, errH);
      max_gradient_relative_error = std::max(max_gradient_relative_error, errG);
      if (errH > 1e-8 || errG > 1e-8) {
        std::cerr << "reconstruction mismatch " << errH << " " << errG << "\n";
        return 2;
      }
      ++checks;
    }
  }
  // Rank-one information, null-space gradient must be projected away.
  Matrix h = Matrix::Zero(); h(0,0) = 4.0;
  Vector gradient = Vector::Zero(); gradient[0] = 8.0; gradient[8] = 1e-8;
  const auto singular = mhe_sensor_fusion::rank_aware::factorize<9>(h, gradient);
  if (!singular.valid || singular.rank != 1 ||
      std::abs(singular.projected_gradient[0] - 8.0) > 1e-10 ||
      std::abs(singular.projected_gradient[8]) > 1e-10 ||
      std::abs(singular.discarded_gradient_norm - 1e-8) > 1e-10) {
    return 3;
  }
  ++checks;
  // Reject an inconsistent large gradient in an unobservable direction:
  // the original linearized quadratic would be unbounded there.
  gradient[8] = 10.0;
  if (mhe_sensor_fusion::rank_aware::factorize<9>(h, gradient).valid) {return 6;}
  ++checks;
  // A strongly indefinite Schur complement is an error, not a valid prior.
  Matrix broken = Matrix::Identity(); broken(8,8) = -0.2;
  if (mhe_sensor_fusion::rank_aware::factorize<9>(broken, Vector::Zero()).valid) {
    return 4;
  }
  ++checks;
  const auto empty = mhe_sensor_fusion::rank_aware::factorize<9>(
    Matrix::Zero(), Vector::Zero());
  if (!empty.valid || empty.rank != 0 || empty.sqrt_info.norm() > 1e-12) {return 5;}
  ++checks;
  std::cout << "rank-aware prior C++ PASS: " << checks
            << " cases, max_H_error=" << max_information_relative_error
            << ", max_gradient_error=" << max_gradient_relative_error << "\n";
  return 0;
}
