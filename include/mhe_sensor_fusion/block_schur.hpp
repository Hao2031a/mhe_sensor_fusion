#pragma once

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <vector>

namespace mhe_sensor_fusion::block_schur
{
// Block-tridiagonal normal equations for a chain of NX-dimensional MHE states.
// A factor is admissible only when all of its parameter blocks belong to one
// state or to two *consecutive* states. This is verified by the caller.
template<int NX>
class Chain
{
public:
  using Matrix = Eigen::Matrix<double, NX, NX>;
  using Vector = Eigen::Matrix<double, NX, 1>;
  explicit Chain(int n)
  : diag_(static_cast<size_t>(n), Matrix::Zero()),
    upper_(static_cast<size_t>(n > 0 ? n - 1 : 0), Matrix::Zero()),
    rhs_(static_cast<size_t>(n), Vector::Zero()) {}

  int size() const { return static_cast<int>(diag_.size()); }
  bool add(int i, int j, const Matrix & h)
  {
    if (i < 0 || j < 0 || i >= size() || j >= size()) {return false;}
    if (i == j) { diag_[static_cast<size_t>(i)] += h; return true; }
    if (j == i + 1) { upper_[static_cast<size_t>(i)] += h; return true; }
    if (i == j + 1) { upper_[static_cast<size_t>(j)] += h.transpose(); return true; }
    return false;
  }
  bool addRhs(int i, const Vector & r)
  {
    if (i < 0 || i >= size()) {return false;}
    rhs_[static_cast<size_t>(i)] += r;
    return true;
  }

  // Eliminate states [0, retained) in O(retained * NX^3), without a dense
  // (retained*NX) x (retained*NX) matrix. The caller forms the prior at the
  // first retained state from h and b. Successful elimination does not mutate
  // the graph, so the caller can commit atomically after all checks.
  bool eliminate(int retained, Matrix & h, Vector & b, double damping = 1e-8) const
  {
    if (retained < 1 || retained >= size() || !(damping >= 0.0)) {return false;}
    Matrix d = diag_.front();
    Vector r = rhs_.front();
    for (int i = 0; i < retained; ++i) {
      d = Matrix(0.5 * (d + d.transpose()));
      // Damping must not make a completely unobserved eliminated state
      // look observable; refuse a zero-information prefix.
      if (!d.allFinite() || d.cwiseAbs().maxCoeff() < 1e-12) {return false;}
      // Regularization applies only to eliminated states, never the boundary.
      d.diagonal().array() += damping;
      Eigen::LDLT<Matrix> ldlt(d);
      if (ldlt.info() != Eigen::Success ||
          !ldlt.vectorD().allFinite() || ldlt.vectorD().minCoeff() <= 0.0) {
        return false;
      }
      const Matrix & e = upper_[static_cast<size_t>(i)];
      // One factorization AND one matrix solve for both Schur RHS.
      Eigen::Matrix<double, NX, NX + 1> joined;
      joined.template leftCols<NX>() = e;
      joined.col(NX) = r;
      const auto solved = ldlt.solve(joined).eval();
      if (!solved.allFinite()) {return false;}
      d = diag_[static_cast<size_t>(i + 1)] -
        e.transpose() * solved.template leftCols<NX>();
      r = rhs_[static_cast<size_t>(i + 1)] - e.transpose() * solved.col(NX);
    }
    h = 0.5 * (d + d.transpose());
    b = r;
    return h.allFinite() && b.allFinite();
  }

  // Rare numerical fallback. Assemble only the eliminated prefix and its
  // boundary (not the full active horizon). O(m^3), disabled on the fast path.
  bool eliminateDense(int retained, Matrix & h, Vector & b, double damping = 1e-8) const
  {
    if (retained < 1 || retained >= size() || !(damping >= 0.0)) {return false;}
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero((retained + 1) * NX, (retained + 1) * NX);
    Eigen::VectorXd g = Eigen::VectorXd::Zero((retained + 1) * NX);
    for (int i = 0; i <= retained; ++i) {
      H.block<NX, NX>(i * NX, i * NX) = diag_[static_cast<size_t>(i)];
      g.segment<NX>(i * NX) = rhs_[static_cast<size_t>(i)];
      if (i < retained) {
        H.block<NX, NX>(i * NX, (i + 1) * NX) = upper_[static_cast<size_t>(i)];
        H.block<NX, NX>((i + 1) * NX, i * NX) = upper_[static_cast<size_t>(i)].transpose();
      }
    }
    const int m = retained * NX;
    Eigen::MatrixXd A = H.topLeftCorner(m, m);
    A.diagonal().array() += damping;
    Eigen::LDLT<Eigen::MatrixXd> ldlt(A);
    if (ldlt.info() != Eigen::Success ||
        !ldlt.vectorD().allFinite() || ldlt.vectorD().minCoeff() <= 0.0) {return false;}
    const Eigen::MatrixXd B = H.block(0, m, m, NX);
    Eigen::MatrixXd joined(m, NX + 1);
    joined.leftCols(NX) = B;
    joined.col(NX) = g.head(m);
    const Eigen::MatrixXd solved = ldlt.solve(joined);
    if (!solved.allFinite()) {return false;}
    h = H.block<NX, NX>(m, m) - B.transpose() * solved.leftCols(NX);
    b = g.tail(NX) - B.transpose() * solved.col(NX);
    h = 0.5 * (h + h.transpose());
    return h.allFinite() && b.allFinite();
  }

private:
  std::vector<Matrix> diag_;
  std::vector<Matrix> upper_;
  std::vector<Vector> rhs_;
};
}  // namespace mhe_sensor_fusion::block_schur
