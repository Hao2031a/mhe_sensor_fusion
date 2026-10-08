// Ceres AutoDiff-vs-analytic test for SO(2) wrapping and exact SE(2) arc.
// Requires the Docker/Kilted Ceres environment; standalone math tests run
// without ROS or Ceres and are separate.
#include <ceres/ceres.h>
#include "mhe_sensor_fusion/analytic_factors.hpp"
#include "mhe_sensor_fusion/se2_yaw_math.hpp"
#include <array>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>

constexpr int K=9;
using mhe_fusion::yaw_math::kPi;
struct AutoProcess {
  std::array<double,K> sigma;
  double dt;
  template<typename T>
  bool operator()(const T *const a,const T *const b,T *r) const {
    T vm=T(0.5)*(a[3]+b[3]);
    T wm=T(0.5)*(a[4]+b[4]);
    T dx,dy;
    mhe_fusion::yaw_math::integrateArc(a[2],vm,wm,T(dt),dx,dy);
    r[0]=(b[0]-a[0]-dx)/T(sigma[0]);
    r[1]=(b[1]-a[1]-dy)/T(sigma[1]);
    r[2]=mhe_fusion::yaw_math::wrapDifference(b[2]-a[2]-wm*T(dt))/T(sigma[2]);
    for(int j=3;j<K;j++)r[j]=(b[j]-a[j])/T(sigma[j]);
    return true;
  }
};
int main() {
  std::mt19937_64 rng(20261008);
  std::uniform_real_distribution<double> rand(-1,1);
  std::array<double,K> sigma{{0.08,0.08,0.04,0.3,0.3,0.02,0.02,0.1,0.1}};
  int tests=0;
  double max_error=0;
  for(int i=0;i<500;i++) {
    double dt=0.002+0.099*(i%99)/99.0;
    std::array<double,K> a{},b{};
    for(int k=0;k<K;k++) {a[k]=rand(rng); b[k]=a[k]+0.1*rand(rng);}
    a[2]=kPi*(2.0*rand(rng));
    b[2]=a[2]+(a[4]+b[4])*0.5*dt+0.02*rand(rng);
    if(i%3==0)b[2]+=2*kPi;
    if(i%3==1)b[2]-=2*kPi;
    mhe_fusion::analytic::ProcessKernel reference(dt,sigma,true,true);
    double residual_ref[K],ja[K*K],jb[K*K];
    if(!reference.evaluate(a.data(),b.data(),residual_ref,ja,jb))throw std::runtime_error("analytic failure");
    auto cost=ceres::AutoDiffCostFunction<AutoProcess,K,K,K>(new AutoProcess{sigma,dt});
    const double* ptrs[]={a.data(),b.data()};
    double res[K],jac0[K*K],jac1[K*K];
    double *jacs[]={jac0,jac1};
    if(!cost.Evaluate(ptrs,res,jacs))throw std::runtime_error("autodiff failure");
    for(int j=0;j<K;j++) {
      if(std::abs(res[j]-residual_ref[j])>1e-10)throw std::runtime_error("res mismatch");
      tests++;
    }
    for(int j=0;j<K*K;j++) {
      double e0=std::abs(jac0[j]-ja[j]),e1=std::abs(jac1[j]-jb[j]);
      max_error=std::max({max_error,e0,e1});
      if(e0>1e-8 || e1>1e-8) throw std::runtime_error("analytic/autodiff Jacobian mismatch");
      tests+=2;
    }
  }
  std::cout<<"CERES_SO2_YAW_PASS checks="<<tests<<" max_error="<<max_error<<"\n";
}
