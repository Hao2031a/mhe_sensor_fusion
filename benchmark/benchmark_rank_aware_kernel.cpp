// Fixed 9x9 prior-construction kernel A/B. NOT a ROS/Ceres/SLAM benchmark.
#include "mhe_sensor_fusion/rank_aware_prior.hpp"
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <vector>
#include <algorithm>

using Mat=Eigen::Matrix<double,9,9>;
using Vec=Eigen::Matrix<double,9,1>;

bool old(const Mat & H, const Vec & b, Mat & A, Vec & c) {
  Eigen::SelfAdjointEigenSolver<Mat> solver(0.5*(H+H.transpose()));
  if (solver.info()!=Eigen::Success) return false;
  Vec eigen=solver.eigenvalues();
  for (int i=0;i<9;++i) eigen[i]=std::max(eigen[i],1e-7);
  Mat reconstructed=solver.eigenvectors()*eigen.asDiagonal()*solver.eigenvectors().transpose();
  Eigen::LLT<Mat> llt(reconstructed);
  if (llt.info()!=Eigen::Success) return false;
  Mat L=llt.matrixL();
  A=L.transpose();
  c=L.triangularView<Eigen::Lower>().solve(b);
  return true;
}

int main(int argc, char**argv) {
  std::string output;
  for (int i=1;i+1<argc;++i) if(std::string(argv[i])=="--output")output=argv[i+1];
  std::mt19937_64 rng(20261008);
  std::normal_distribution<double> N;
  std::vector<Mat> H;std::vector<Vec>b;
  for(int i=0;i<128;++i){
    Mat m;Vec g;
    for(int k=0;k<81;++k) m.data()[k]=N(rng);
    for(int k=0;k<9;++k)g[k]=N(rng);
    H.push_back(m.transpose()*m+Mat::Identity());
    b.push_back(g);
  }
  constexpr int iterations=12;
  constexpr int batch=128;
  volatile double checksum=0.0;
  auto benchmark=[&](bool new_mode){
    std::vector<double> ns;
    for(int trial=0;trial<iterations;++trial){
      auto t0=std::chrono::steady_clock::now();
      for(int i=0;i<batch;++i){
        if(new_mode) {
          const auto result=mhe_sensor_fusion::rank_aware::factorize<9>(H[i],b[i]);
          if(!result.valid) return std::vector<double>{};
          checksum+=result.offset[0];
        }else{
          Mat A;Vec c;
          if(!old(H[i],b[i],A,c)) return std::vector<double>{};
          checksum+=c[0];
        }
      }
      const double elapsed=std::chrono::duration<double,std::nano>(
        std::chrono::steady_clock::now()-t0).count()/batch;
      ns.push_back(elapsed);
    }
    std::sort(ns.begin(),ns.end());
    return ns;
  };
  const auto legacy=benchmark(false);
  const auto ranked=benchmark(true);
  if(legacy.empty()||ranked.empty()) return 1;
  const double base=legacy[iterations/2],current=ranked[iterations/2];
  std::cout<<"C++ 9x9 microbenchmark, median nanoseconds per prior: legacy="<<base
    <<", rank_aware="<<current<<", ratio="<<current/base<<" (NOT ROS walltime)\n";
  if(!output.empty()){
    std::ofstream o(output);
    o<<"{\"type\":\"local_cpp_9x9_microbenchmark_not_ros\",\"legacy_median_ns\":"
      <<base<<",\"rank_aware_median_ns\":"<<current
      <<",\"ratio\":"<<current/base<<",\"hard_gate\":false,\"pass\":true}\n";
  }
  return std::isfinite(checksum)?0:2;
}
