#pragma once

// Jacobi-scaled selected covariance from a frozen Ceres CRS Jacobian.
// Used ONLY on the asynchronous worker, never in the 100 Hz callback.
// LLT is guarded by reciprocal condition; SVD acts on the ORIGINAL scaled
// Jacobian (not J^T J) when LLT is unreliable. We refuse rank-deficient
// snapshots rather than assign fictitiously small nullspace variance.
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <vector>

namespace mhe_fusion::scaled_cov
{
struct Config
{
  double minimum_rcond{1e-10};
  double svd_relative_cutoff{1e-11};
  double maximum_svd_condition{1e10};
  bool allow_svd{true};
};

struct Solution
{
  bool valid{false};
  int method{0};  // 1: LLT, 2: SVD, 3: rank/condition rejection
  int rank{0};
  double reciprocal_condition{0.0};
  Eigen::MatrixXd covariance;
};

inline Solution selected(
  const Eigen::Ref<const Eigen::MatrixXd> & jacobian, int tail,
  const Config & config = Config{})
{
  Solution output;
  output.method = 3;
  const Eigen::Index rows = jacobian.rows(), cols = jacobian.cols();
  if (rows < cols || cols <= 0 || cols > 512 || tail <= 0 || tail > cols ||
      !jacobian.allFinite()) {return output;}
  const Eigen::VectorXd diagonal = jacobian.array().square().colwise().sum().transpose();
  if (!diagonal.allFinite() || diagonal.minCoeff() <= 1e-20) {return output;}

  // Diagonal equilibration ensures meter, radian and slip components do not
  // dominate condition detection only because their numeric units differ.
  Eigen::VectorXd scales = diagonal.array().sqrt().inverse();
  Eigen::MatrixXd normalized = jacobian * scales.asDiagonal();
  Eigen::MatrixXd hessian = normalized.transpose() * normalized;
  hessian = 0.5 * (hessian + hessian.transpose()).eval();
  Eigen::LLT<Eigen::MatrixXd> llt(hessian);
  if (llt.info() == Eigen::Success) {
    const double rcond = llt.rcond();
    output.reciprocal_condition = std::isfinite(rcond) ? rcond : 0.0;
    if (std::isfinite(rcond) && rcond >= config.minimum_rcond) {
      Eigen::MatrixXd rhs = Eigen::MatrixXd::Zero(cols, tail);
      rhs.bottomRows(tail).setIdentity();
      Eigen::MatrixXd inverse_columns = llt.solve(rhs);
      const Eigen::VectorXd tail_scales = scales.tail(tail);
      Eigen::MatrixXd cov = tail_scales.asDiagonal() *
        inverse_columns.bottomRows(tail) * tail_scales.asDiagonal();
      cov = 0.5 * (cov + cov.transpose()).eval();
      if (cov.allFinite() && (cov.diagonal().array() > 0).all()) {
        output.valid = true;
        output.method = 1;
        output.rank = static_cast<int>(cols);
        output.covariance = std::move(cov);
        return output;
      }
    }
  }
  if (!config.allow_svd) {return output;}

  // No pseudoinverse in a rank-deficient system: it would report ZERO
  // uncertainty in the nullspace, a dangerous overconfidence in yaw/slip.
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(normalized, Eigen::ComputeThinV);
  if (svd.info() != Eigen::Success || svd.singularValues().size() != cols) {
    return output;
  }
  const Eigen::VectorXd sig = svd.singularValues();
  const double maximum = sig[0];
  const double minimum = sig[cols - 1];
  if (!(maximum > 0.0) || !std::isfinite(minimum)) {return output;}
  const double rel = minimum / maximum;
  const double effective_cutoff = std::max(config.svd_relative_cutoff, 0.0);
  output.rank = static_cast<int>((sig.array() > maximum * effective_cutoff).count());
  output.reciprocal_condition = rel * rel;
  if (output.rank != cols || rel < 1.0 / config.maximum_svd_condition) {
    return output;
  }
  Eigen::MatrixXd v_tail = svd.matrixV().bottomRows(tail);
  Eigen::VectorXd inverse_squares = sig.array().square().inverse();
  Eigen::MatrixXd cov = v_tail * inverse_squares.asDiagonal() * v_tail.transpose();
  const Eigen::VectorXd tail_scales = scales.tail(tail);
  cov = tail_scales.asDiagonal() * cov * tail_scales.asDiagonal();
  cov = 0.5 * (cov + cov.transpose()).eval();
  if (!cov.allFinite() || (cov.diagonal().array() <= 0).any()) {return output;}
  output.valid = true;
  output.method = 2;
  output.covariance = std::move(cov);
  return output;
}
} // namespace mhe_fusion::scaled_cov
