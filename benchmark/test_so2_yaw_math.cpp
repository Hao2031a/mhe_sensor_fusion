#include "mhe_sensor_fusion/se2_yaw_math.hpp"
#include "mhe_sensor_fusion/analytic_factors.hpp"
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <random>
#include <stdexcept>
#include <vector>

using mhe_fusion::yaw_math::kPi;
using mhe_fusion::analytic::ProcessKernel;
constexpr int N = 9;
static void check(bool good, const char * message) {
  if (!good) {throw std::runtime_error(message);}
}
int main() {
  std::mt19937_64 rng(20261008);
  std::uniform_real_distribution<double> yaw(-kPi,kPi), speed(-1.0,1.0), turn(-6.0,6.0);
  std::uniform_real_distribution<double> delta(0.001,0.10);
  std::array<double,N> sigma{{0.1,0.1,0.04,0.3,0.3,0.01,0.02,0.1,0.1}};
  int checks=0;
  double max_jac_error=0.0, max_arc_error=0.0, max_so2_error=0.0;
  for(int i=0;i<2400;i++) {
    double t=delta(rng), theta=yaw(rng), v=speed(rng), w=turn(rng);
    if(i%17==0) w=0.0;
    if(i%17==1) w=1e-12;
    const auto a=mhe_fusion::yaw_math::arcWithDerivatives(theta,v,w,t);
    double dx=0,dy=0;
    mhe_fusion::yaw_math::integrateArc(theta,v,w,t,dx,dy);
    check(std::abs(a.dx-dx)<1e-13 && std::abs(a.dy-dy)<1e-13,"arc self-consistency");
    // Closed form equivalent to integrating tiny midpoint steps (independent numerical reference).
    double xx=0,yy=0;
    for(int j=0;j<128;j++) {
      double th=theta+(j+0.5)*(w*t/128.0);
      xx+=v*t/128.0*std::cos(th);
      yy+=v*t/128.0*std::sin(th);
    }
    double arc_err=std::hypot(xx-a.dx, yy-a.dy);
    max_arc_error=std::max(max_arc_error,arc_err);
    check(arc_err<2e-7, "arc vs fine-grained quadrature");
    checks+=3;

    std::array<double,N> s0{},s1{};
    s0[0]=speed(rng);s0[1]=speed(rng);s0[2]=theta;
    s0[3]=v;s0[4]=w;s0[5]=speed(rng)*0.01;
    s1=s0;
    s1[3]+=speed(rng)*0.1;s1[4]+=speed(rng)*0.2;
    double vm=0.5*(s0[3]+s1[3]), wm=0.5*(s0[4]+s1[4]);
    mhe_fusion::yaw_math::integrateArc(theta,vm,wm,t,dx,dy);
    s1[0]+=dx+speed(rng)*0.001;
    s1[1]+=dy+speed(rng)*0.001;
    s1[2]=s0[2]+wm*t+speed(rng)*0.01;
    if(i%4==0)s1[2]+=2.0*kPi;
    if(i%4==1)s1[2]-=2.0*kPi;
    ProcessKernel kernel(t,sigma,true,true);
    double res[N],j0[N*N],j1[N*N];
    check(kernel.evaluate(s0.data(),s1.data(),res,j0,j1),"kernel evaluation");
    const double h=1e-6;
    for(int b=0;b<2;b++) {
      for(int k=0;k<N;k++) {
        auto lhs=s0,rhs=s1;
        auto & state=(b==0?lhs:rhs);
        state[k]+=h;
        double rp[N];
        check(kernel.evaluate(lhs.data(),rhs.data(),rp,nullptr,nullptr),"jac plus");
        state[k]-=2*h;
        double rm[N];
        check(kernel.evaluate(lhs.data(),rhs.data(),rm,nullptr,nullptr),"jac minus");
        for(int r=0;r<N;r++) {
          const double approx=(rp[r]-rm[r])/(2*h);
          const double analytical=(b==0?j0:j1)[r*N+k];
          const double err=std::abs(approx-analytical);
          max_jac_error=std::max(max_jac_error,err);
          check(err<1e-6+1e-7*std::abs(analytical),"jacobian mismatch");
          checks++;
        }
      }
    }
    // SO(2) process residual invariant under representation +/- 2pi.
    auto s2=s1;
    s2[2]+=2*kPi;
    double rr[N];
    check(kernel.evaluate(s0.data(),s2.data(),rr,nullptr,nullptr),"wrapped residual eval");
    double so2err=std::abs(rr[2]-res[2]);
    max_so2_error=std::max(max_so2_error,so2err);
    check(so2err<1e-12,"angle periodicity");
    checks+=2;
  }
  // Crossing +pi/-pi must not create a giant artificial innovation.
  {
    std::array<double,N> x0{},x1{};
    x0[2]=kPi-0.001; x1[2]=-kPi+0.001;
    ProcessKernel old(0.02,sigma,false,false), modern(0.02,sigma,true,true);
    double r_old[N],r_new[N];
    old.evaluate(x0.data(),x1.data(),r_old,nullptr,nullptr);
    modern.evaluate(x0.data(),x1.data(),r_new,nullptr,nullptr);
    check(std::abs(r_new[2]-0.002/sigma[2])<1e-10,"correct +/-pi crossing");
    check(std::abs(r_old[2])>100,"baseline detects large false angle residual");
    checks+=2;
  }
  std::cout<<"SO2_YAW_MATH_PASS checks="<<checks<<" max_jac_error="<<max_jac_error
           <<" max_arc_error="<<max_arc_error<<" max_periodic_error="<<max_so2_error<<"\n";
}
