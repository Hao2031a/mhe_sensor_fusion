// Real Ceres integration A/B regression for analytic process/wheel factors.
// Generates the same 13-state noisy horizon, same costs, same Ceres settings.
// Wall-clock timing is diagnostic only: never gate on hardware speed ratios.
#include "mhe_sensor_fusion/analytic_factors.hpp"
#include <ceres/ceres.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>
namespace an = mhe_fusion::analytic;
constexpr int N = 9, H = 13;
using Vec = std::array<double, N>;
struct ProcRef {
  double dt; Vec sigma;
  template <class T> bool operator()(const T *a, const T *b, T *r) const {
    T vbar = T(0.5)*(a[an::kV]+b[an::kV]);
    T wbar = T(0.5)*(a[an::kW]+b[an::kW]);
    T mid = a[an::kYaw]+T(0.5*dt)*wbar;
    r[an::kX]=(b[an::kX]-(a[an::kX]+T(dt)*vbar*cos(mid)))/T(sigma[an::kX]);
    r[an::kY]=(b[an::kY]-(a[an::kY]+T(dt)*vbar*sin(mid)))/T(sigma[an::kY]);
    r[an::kYaw]=(b[an::kYaw]-(a[an::kYaw]+T(dt)*wbar))/T(sigma[an::kYaw]);
    for (int i=an::kV;i<N;++i) r[i]=(b[i]-a[i])/T(sigma[i]);
    return true;
  }
};
struct WheelRef {
  double left,right,sep,sl,sr;
  template<class T> bool operator()(const T*x,T*r) const {
    T half=T(0.5*sep);
    r[0]=((T(1)+x[an::kSl])*(x[an::kV]-half*x[an::kW])-T(left))/T(sl);
    r[1]=((T(1)+x[an::kSr])*(x[an::kV]+half*x[an::kW])-T(right))/T(sr);
    return true;
  }
};
class ProcAnalytic final:public ceres::SizedCostFunction<N,N,N> {
  an::ProcessKernel kernel;
public:
  ProcAnalytic(double dt,const Vec&sigma):kernel(dt,sigma) {}
  bool Evaluate(double const*const *p,double*r,double**j)const override {
    return kernel.evaluate(p[0],p[1],r,j?j[0]:nullptr,j?j[1]:nullptr);
  }
};
class WheelAnalytic final:public ceres::SizedCostFunction<2,N> {
  an::WheelKernel kernel;
public:
  WheelAnalytic(double l,double r,double sep,double sl,double sr):kernel(l,r,sep,sl,sr){}
  bool Evaluate(double const*const *p,double*r,double**j)const override {
    return kernel.evaluate(p[0],r,j?j[0]:nullptr);
  }
};
struct Prior {
  Vec ref; double s;
  template<class T>bool operator()(const T*x,T*r)const {
    for(int i=0;i<N;++i)r[i]=(x[i]-T(ref[i]))/T(s);
    return true;
  }
};
struct Result {std::vector<Vec> states; double cost; double ms;};
Result solve(bool analytic,int seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.,1.);
  std::vector<Vec> truth(H), estimate(H);
  for(int k=0;k<H;++k) {
    double t=0.018*k;
    truth[k]={0.2+0.27*t,0.05+0.12*t*t,0.5+0.16*t,0.27+0.02*t,
              0.16+0.03*t,0.005,0.01,0.008,-0.006};
    estimate[k]=truth[k];
    for(int j=0;j<N;++j)estimate[k][j] += (j<3 ? 0.005 : 0.015)*noise(rng);
  }
  const Vec sigma={0.04,0.04,0.02,0.10,0.20,0.03,0.03,0.02,0.02};
  ceres::Problem problem;
  for(int k=0;k<H;++k) {
    problem.AddParameterBlock(estimate[k].data(), N);
    const double half=.15;
    double vl=truth[k][an::kV]-half*truth[k][an::kW]+.001*noise(rng);
    double vr=truth[k][an::kV]+half*truth[k][an::kW]+.001*noise(rng);
    ceres::CostFunction *wheel = analytic ?
      static_cast<ceres::CostFunction*>(new WheelAnalytic(vl,vr,0.3,0.025,0.025)):
      static_cast<ceres::CostFunction*>(new ceres::AutoDiffCostFunction<WheelRef,2,N>(
        new WheelRef{vl,vr,.3,.025,.025}));
    problem.AddResidualBlock(wheel,new ceres::HuberLoss(1.5),estimate[k].data());
    if(k>0) {
      double dt=.018;
      ceres::CostFunction *proc = analytic ?
        static_cast<ceres::CostFunction*>(new ProcAnalytic(dt,sigma)):
        static_cast<ceres::CostFunction*>(new ceres::AutoDiffCostFunction<ProcRef,N,N,N>(
          new ProcRef{dt,sigma}));
      problem.AddResidualBlock(proc,nullptr,estimate[k-1].data(),estimate[k].data());
    }
    // Strong anchors for first state, weak per-state priors to avoid gauge
    // ambiguity with the wheel-only synthetic measurement model.
    auto *prior=new ceres::AutoDiffCostFunction<Prior,N,N>(new Prior{truth[k],k==0?0.005:1.0});
    problem.AddResidualBlock(prior,nullptr,estimate[k].data());
  }
  ceres::Solver::Options opts;
  opts.linear_solver_type=ceres::DENSE_QR;
  opts.max_num_iterations=6;
  opts.num_threads=1;
  opts.minimizer_progress_to_stdout=false;
  ceres::Solver::Summary sum;
  auto started=std::chrono::steady_clock::now();
  ceres::Solve(opts,&problem,&sum);
  double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
  if(!sum.IsSolutionUsable())throw std::runtime_error("Ceres failed");
  return {estimate,sum.final_cost,ms};
}
int main(int argc,char**argv) {
  std::string path;
  for(int i=1;i+1<argc;++i)if(std::string(argv[i])=="--output")path=argv[++i];
  std::vector<double> ana,ref;
  double worst_state=0,worst_cost=0;
  for(int seed=1;seed<=24;++seed) {
    auto a=solve(true,seed), b=solve(false,seed);
    ana.push_back(a.ms); ref.push_back(b.ms);
    worst_cost=std::max(worst_cost,std::abs(a.cost-b.cost));
    for(size_t k=0;k<a.states.size();++k)for(int j=0;j<N;++j)
      worst_state=std::max(worst_state,std::abs(a.states[k][j]-b.states[k][j]));
  }
  std::sort(ana.begin(),ana.end());std::sort(ref.begin(),ref.end());
  const auto median=[](const std::vector<double>&v){return (v[11]+v[12])*0.5;};
  const bool pass=worst_state<1e-5 && worst_cost<1e-7;
  std::cout << (pass?"PASS":"FAIL") << " 24 Ceres A/B horizons: max_state_delta="
    << worst_state << " max_final_cost_delta=" << worst_cost
    << " median_analytic_ms=" << median(ana) << " median_autodiff_ms=" << median(ref) << "\n";
  if(!path.empty()){
    std::ofstream f(path);
    f<<std::setprecision(16)<<"{\n  \"pass\": "<<(pass?"true":"false")
      <<",\n  \"samples\": 24,\n  \"max_state_delta\": "<<worst_state
      <<",\n  \"max_final_cost_delta\": "<<worst_cost
      <<",\n  \"median_analytic_solver_ms\": "<<median(ana)
      <<",\n  \"median_autodiff_solver_ms\": "<<median(ref)
      <<",\n  \"hardware_timing_is_advisory\": true\n}\n";
  }
  return pass?0:1;
}
