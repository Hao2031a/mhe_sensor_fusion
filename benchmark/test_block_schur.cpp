#include "mhe_sensor_fusion/block_schur.hpp"
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <cassert>
#include <cmath>
#include <iostream>
#include <random>

using Chain = mhe_sensor_fusion::block_schur::Chain<9>;
using Mat = Chain::Matrix;
using Vec = Chain::Vector;

int main()
{
  std::mt19937 gen(20261008);
  std::normal_distribution<double> norm(0.0, 1.0);
  int count = 0;
  for (int n = 2; n <= 9; ++n) {
    for (int trial = 0; trial < 24; ++trial) {
      Chain chain(n);
      Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n * 9, n * 9);
      Eigen::VectorXd b = Eigen::VectorXd::Zero(n * 9);
      auto randMat = [&]() {
        Mat m;
        for (int k = 0; k < m.size(); ++k) {m.data()[k] = norm(gen);}
        return m;
      };
      for (int i = 0; i < n; ++i) {
        Mat a = randMat();
        Mat d = a.transpose() * a + 30.0 * Mat::Identity();
        Vec r;
        for (int k = 0; k < 9; ++k) {r[k] = norm(gen);}
        if (!chain.add(i, i, d)) {return 2;}
        if (!chain.addRhs(i, r)) {return 3;}
        H.block<9, 9>(i * 9, i * 9) += d;
        b.segment<9>(i * 9) += r;
        if (i > 0) {
          Mat e = 0.04 * randMat();
          if (!chain.add(i - 1, i, e)) {return 4;}
          H.block<9, 9>((i - 1) * 9, i * 9) += e;
          H.block<9, 9>(i * 9, (i - 1) * 9) += e.transpose();
        }
      }
      for (int remove = 1; remove < n; ++remove) {
        Mat hm;
        Vec bm;
        if (!chain.eliminate(remove, hm, bm)) {return 5;}
        Mat dense_h;
        Vec dense_b;
        if (!chain.eliminateDense(remove, dense_h, dense_b)) {return 8;}
        if ((hm - dense_h).norm()>1e-8 || (bm-dense_b).norm()>1e-8) {return 9;}
        const int nr = 9 * remove;
        Eigen::MatrixXd haa = H.topLeftCorner(nr, nr);
        haa.diagonal().array() += 1e-8;
        Eigen::LDLT<Eigen::MatrixXd> ldl(haa);
        const Eigen::MatrixXd hab = H.block(0, nr, nr, 9);
        const Mat ref_h = H.block<9, 9>(nr, nr) - hab.transpose() * ldl.solve(hab);
        const Vec ref_b = b.segment<9>(nr) - hab.transpose() * ldl.solve(b.head(nr));
        if ((hm - ref_h).norm() > 1e-8 || (bm - ref_b).norm() > 1e-8) {
          std::cerr << "Schur mismatch n=" << n << " remove=" << remove << "\n";
          return 1;
        }
        ++count;
      }
      // Non-local factor cannot be represented by a chain edge.
      if (n > 2 && chain.add(0, 2, Mat::Identity())) {return 6;}
    }
  }
  Chain bad(2);
  Mat hm;
  Vec bm;
  if (bad.eliminate(1, hm, bm)) {return 7;}
  std::cout << "PASS " << count << " block Schur vs dense marginal checks\n";
}
