// Worker-kernel A/B on the SAME frozen synthetic sparse Jacobian.
// This measures neither ROS middleware nor Gazebo and is not WCET.
#include "mhe_sensor_fusion/fast_covariance.hpp"
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>
using mhe_fusion::fast_cov::Snapshot;
using mhe_fusion::fast_cov::compute;
static double percentile(std::vector<double> v, double q){std::sort(v.begin(),v.end());return v[std::min(v.size()-1,static_cast<size_t>(q*(v.size()-1)))];}
int main(int argc,char**argv){
  constexpr int n=13*9;
  Snapshot s; s.rows=2*n-1; s.cols=n; s.tail_size=9;
  s.row_offsets.push_back(0);
  for(int i=0;i<n;++i){
    s.column_indices.push_back(i);s.values.push_back(2.0+0.01*(i%9));
    s.row_offsets.push_back(static_cast<int>(s.values.size()));
  }
  for(int i=1;i<n;++i){
    s.column_indices.push_back(i-1);s.column_indices.push_back(i);
    s.values.push_back(-1.);s.values.push_back(1.);
    s.row_offsets.push_back(static_cast<int>(s.values.size()));
  }
  auto legacy=s; auto scaled=s; scaled.jacobi_scaled_enabled=true;
  constexpr int trials=120;
  std::vector<double> la,lb;
  // Warmup both paths
  for(int i=0;i<5;++i){if(!compute(legacy).valid||!compute(scaled).valid)throw std::runtime_error("warmup covariance failed");}
  for(int i=0;i<trials;++i){
    // Alternate order to reduce systematic CPU-cache order bias.
    auto timed=[](const Snapshot & input){auto t=std::chrono::steady_clock::now();
      auto out=compute(input);
      if(!out.valid)throw std::runtime_error("covariance invalid");
      return std::pair<double,decltype(out)>{std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t).count(),std::move(out)};};
    auto a=(i%2)?timed(scaled):timed(legacy);
    auto b=(i%2)?timed(legacy):timed(scaled);
    auto & old=(i%2)?b:a;
    auto & modern=(i%2)?a:b;
    if(old.second.covariance.size()!=modern.second.covariance.size())throw std::runtime_error("size mismatch");
    for(size_t j=0;j<old.second.covariance.size();++j){
      if(std::abs(old.second.covariance[j]-modern.second.covariance[j])>2e-6){
        throw std::runtime_error("A/B covariance deviation");
      }
    }
    la.push_back(old.first);lb.push_back(modern.first);
  }
  double op50=percentile(la,.5),op99=percentile(la,.99);
  double np50=percentile(lb,.5),np99=percentile(lb,.99);
  std::cout<<"Synthetic covariance 117x (120 trials) old p50="<<op50
    <<" p99="<<op99<<" scaled p50="<<np50<<" p99="<<np99<<" ms\n";
  if(argc==3&&std::string(argv[1])=="--output"){
    std::ofstream f(argv[2]);
    f<<"{\"pass\":true,\"kind\":\"offline_synthetic_covariance_kernel\",\"trials\":120,"
     <<"\"legacy_p50_ms\":"<<op50<<",\"legacy_p99_ms\":"<<op99
     <<",\"scaled_p50_ms\":"<<np50<<",\"scaled_p99_ms\":"<<np99<<"}\n";
  }
}
