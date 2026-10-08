#include "mhe_sensor_fusion/rank_aware_prior.hpp"
#include "mhe_sensor_fusion/block_schur.hpp"
#include <Eigen/Cholesky>
#include <iostream>
#include <random>
#include <cmath>

using Chain = mhe_sensor_fusion::block_schur::Chain<9>;
using Mat = Chain::Matrix;
using Vec = Chain::Vector;

int main()
{
  std::mt19937 gen(20261008);
  std::normal_distribution<double> normal;
  int cases = 0;
  double max_error = 0.0;
  for (int num = 2; num <= 8; ++num) {
    for (int trial = 0; trial < 24; ++trial) {
      Chain chain(num);
      // Build PSD block-tridiagonal factor normal equations with sparse
      // cross-state process factors. The latest state has all 9 variables.
      for (int i = 0; i < num; ++i) {
        Mat local = Mat::Zero();
        for (int r = 0; r < 9; ++r) for (int c = 0; c < 9; ++c)
          local(r,c) = normal(gen);
        Mat D = local.transpose()*local + Mat::Identity()*2.0;
        if (!chain.add(i,i,D)) return 1;
        Vec g;
        for (int j = 0; j < 9; ++j) g[j] = normal(gen);
        if (!chain.addRhs(i,g)) return 2;
        if (i > 0) {
          Mat edge = Mat::Zero();
          for (int r = 0; r < 9; ++r) for (int c = 0; c < 9; ++c)
            edge(r,c)=normal(gen)*0.03;
          if (!chain.add(i-1,i,edge)) return 3;
        }
      }
      for (int eliminated = 1; eliminated < num; ++eliminated) {
        Mat H; Vec b;
        if (!chain.eliminate(eliminated,H,b)) return 4;
        const auto p = mhe_sensor_fusion::rank_aware::factorize<9>(H,b);
        if (!p.valid || p.rank != 9) return 5;
        double e = (H - p.sqrt_info.transpose()*p.sqrt_info).norm()/H.norm();
        double eg = (b - p.sqrt_info.transpose()*p.offset).norm()/std::max(1.0,b.norm());
        max_error = std::max(max_error, std::max(e,eg));
        if (e > 1e-8 || eg > 1e-8) return 6;
        ++cases;
      }
    }
  }
  std::cout << "PASS block Schur -> square-root prior: " << cases <<
    " cases; max relative error=" << max_error << "\n";
  return 0;
}
