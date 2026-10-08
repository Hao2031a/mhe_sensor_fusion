#pragma once

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <limits>

namespace mhe_sensor_fusion::rank_aware
{
// Form a square-root representation of a possibly rank-deficient marginal
// information system: 1/2 dx^T H dx + b^T dx.
//
// State scaling makes the relative eigenvalue threshold dimensionless despite
// heterogeneous state coordinates [m, rad, m/s, rad/s, bias, slip].  Eigenmodes
// beneath the threshold are NOT given artificial information.  The gradient
// is projected onto the retained range as required for a bounded quadratic.
// No covariance inverse or dense normal-equation inverse is formed.
//
// A is a full NX x NX matrix, with zero rows for null modes: this preserves
// compatibility with a fixed-size Ceres prior residual and its Jacobian.
template<int NX>
struct Result
{
  using Mat = Eigen::Matrix<double, NX, NX>;
  using Vec = Eigen::Matrix<double, NX, 1>;
  Mat sqrt_info = Mat::Zero();
  Vec offset = Vec::Zero();
  Mat projected_information = Mat::Zero();
  Vec projected_gradient = Vec::Zero();
  int rank = 0;
  double condition = 1.0;
  double discarded_gradient_norm = 0.0;
  double max_negative_eigenvalue = 0.0;
  bool valid = false;
};

template<int NX>
Result<NX> factorize(
  const Eigen::Matrix<double, NX, NX> & information,
  const Eigen::Matrix<double, NX, 1> & gradient,
  double relative_cutoff = 1e-9,
  double negative_relative_tolerance = 1e-7,
  double max_discarded_gradient_fraction = 1e-4)
{
  using Mat = Eigen::Matrix<double, NX, NX>;
  using Vec = Eigen::Matrix<double, NX, 1>;
  Result<NX> out;
  if (!information.allFinite() || !gradient.allFinite() ||
      !std::isfinite(relative_cutoff) || relative_cutoff <= 0.0 ||
      relative_cutoff >= 1.0 || !std::isfinite(negative_relative_tolerance) ||
      negative_relative_tolerance < 0.0 ||
      !std::isfinite(max_discarded_gradient_fraction) ||
      max_discarded_gradient_fraction < 0.0) { return out; }

  const Mat H = 0.5 * (information + information.transpose());
  // Jacobi diagonal equilibration: z = D^{-1} dx; H_scaled = D H D.
  // The small relative floor only stabilizes scaling, and does NOT add
  // diagonal information to the actual Hessian.
  const double max_diag = H.diagonal().cwiseAbs().maxCoeff();
  const double diagonal_floor = std::max(max_diag * 1e-12, 1e-16);
  Vec D;
  for (int j = 0; j < NX; ++j) {
    D[j] = 1.0 / std::sqrt(std::max(std::abs(H(j, j)), diagonal_floor));
  }
  const Mat normalized = D.asDiagonal() * H * D.asDiagonal();
  if (!normalized.allFinite()) { return out; }
  Eigen::SelfAdjointEigenSolver<Mat> eig(normalized);
  if (eig.info() != Eigen::Success || !eig.eigenvalues().allFinite()) { return out; }
  const Vec & values = eig.eigenvalues();
  const Mat & directions = eig.eigenvectors();
  const double max_eigen = std::max(0.0, values.maxCoeff());
  if (!(max_eigen > 0.0)) {
    // Completely unobservable prior: do not invent information or a shift.
    out.discarded_gradient_norm = gradient.norm();
    out.valid = out.discarded_gradient_norm <= max_discarded_gradient_fraction *
      std::max(1.0, gradient.norm());
    return out;
  }
  out.max_negative_eigenvalue = std::max(0.0, -values.minCoeff());
  if (values.minCoeff() < -negative_relative_tolerance * max_eigen) {
    // Strongly indefinite information indicates a broken Schur/linearization.
    // Fail safely instead of passing a fictitious positive prior to Ceres.
    return out;
  }

  const double threshold = relative_cutoff * max_eigen;
  const Vec normalized_gradient = D.cwiseProduct(gradient);
  Vec retained = Vec::Zero();
  double min_retained = std::numeric_limits<double>::infinity();
  for (int k = 0; k < NX; ++k) {
    const double lambda = values[k];
    if (lambda <= threshold) { continue; }
    const double root = std::sqrt(lambda);
    // A = diag(sqrt(lambda)) * Q^T * D^{-1}.
    out.sqrt_info.row(k) = root * directions.col(k).transpose() * D.cwiseInverse().asDiagonal();
    out.offset[k] = directions.col(k).dot(normalized_gradient) / root;
    retained[k] = lambda;
    min_retained = std::min(min_retained, lambda);
    ++out.rank;
  }
  out.projected_information = out.sqrt_info.transpose() * out.sqrt_info;
  out.projected_gradient = out.sqrt_info.transpose() * out.offset;
  out.discarded_gradient_norm = (gradient - out.projected_gradient).norm();
  if (out.rank > 0) { out.condition = max_eigen / min_retained; }
  // An inconsistent linear term along the nullspace would make the original
  // singular quadratic unbounded. Never silently discard a large term.
  const bool nullspace_gradient_consistent = out.discarded_gradient_norm <=
    max_discarded_gradient_fraction * std::max(1.0, gradient.norm());
  out.valid = nullspace_gradient_consistent &&
    out.sqrt_info.allFinite() && out.offset.allFinite() &&
    out.projected_gradient.allFinite() && out.projected_information.allFinite();
  return out;
}
}  // namespace mhe_sensor_fusion::rank_aware
