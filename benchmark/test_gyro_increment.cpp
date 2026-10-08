#include "mhe_sensor_fusion/gyro_increment.hpp"
#include <array>
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
using mhe_sensor_fusion::gyro_increment::Kernel;
using mhe_sensor_fusion::gyro_increment::sigmaAngle;
using mhe_sensor_fusion::gyro_increment::sigmaAngleBiasBridge;
using mhe_sensor_fusion::gyro_increment::wrap;
static void check(bool okay, const char * msg) {
  if (!okay) throw std::runtime_error(msg);
}
int main() {
  constexpr int N = 9;
  constexpr double PI = 3.1415926535897932384626433832795;
  std::mt19937_64 rng(9123);
  std::uniform_real_distribution<double> angle(-10*PI,10*PI);
  std::uniform_real_distribution<double> rate(-4,4);
  std::uniform_real_distribution<double> bias(-0.35,0.35);
  std::uniform_real_distribution<double> dt_dist(0.001,0.030);
  int jac_checks = 0;
  for (int k=0;k<3500;++k) {
    double dt = dt_dist(rng), gyro = rate(rng);
    std::array<double,N> a{},b{};
    a[2] = angle(rng);a[5]=bias(rng);b[5]=bias(rng);
    b[2] = a[2] + gyro*dt - (a[5]+b[5])*dt*0.5 + (k%7-3)*0.00002;
    if (k%4==0) b[2] -= 2*PI;
    if (k%4==1) b[2] += 2*PI;
    Kernel kernel(gyro,dt,0.012,0.12,(k%2==0) ? 0.04 : 0.0);
    double r[1],j0[N],j1[N];
    check(kernel.evaluate(a.data(),b.data(),r,j0,j1),"valid kernel rejected");
    check(std::isfinite(r[0]),"nonfinite residual");
    const double h = 1.0e-5;
    for (int j=0;j<2;++j) {
      auto &state = j==0?a:b;
      auto * jac=j==0?j0:j1;
      for (int i=0;i<N;++i) {
        state[i]+=h; double rp[1];kernel.evaluate(a.data(),b.data(),rp);
        state[i]-=2*h; double rm[1];kernel.evaluate(a.data(),b.data(),rm);
        state[i]+=h;
        const double fd=(rp[0]-rm[0])/(2*h);
        if (!(std::abs(fd-jac[i])<3e-6+2e-6*std::abs(jac[i]))) { std::cerr<<"k="<<k<<" j="<<j<<" i="<<i<<" fd="<<fd<<" jac="<<jac[i]<<" raw="<<(b[2]-a[2]-dt*(gyro-.5*(a[5]+b[5])))<<" r="<<r[0]<<"\n"; throw std::runtime_error("bad analytic Jacobian"); }
        ++jac_checks;
      }
    }
  }
  // The same angular motion has the same SO(2) residual regardless of 2pi turns.
  Kernel test(1.0,0.02,0.01,0.12);
  std::array<double,N> a{}, b{}; a[2]=PI-0.01; b[2]=-PI+0.01;
  double r[1];test.evaluate(a.data(),b.data(),r);
  check(std::abs(r[0])<1e-12,"SO2 wrap failed");
  check(std::abs(sigmaAngle(.02,.01,.12)-.02*std::hypot(.01,.12))<1e-14,"sigma model mismatch");
  // Conditional on endpoint biases, Brownian-bridge integral var=q^2*dt^3/12.
  for (double dt : {0.001, 0.01, 0.03, 0.1}) {
    for (double q : {0.0, 0.001, 0.02, 0.4}) {
      const double expected_var = dt*dt*(0.012*0.012+0.12*0.12)
        + q*q*dt*dt*dt/12.0;
      const double predicted = sigmaAngleBiasBridge(dt, 0.012, 0.12, q);
      check(std::abs(predicted*predicted-expected_var) <=
        1e-13*std::max(1.0, expected_var), "bridge variance mismatch");
    }
  }
  check(std::isnan(sigmaAngleBiasBridge(.01,.01,.12,-.1)),"negative q accepted");
  check(std::isnan(sigmaAngleBiasBridge(.01,.01,.12,NAN)),"nonfinite q accepted");
  check(wrap(20*PI+.12) > .119999 && wrap(20*PI+.12) <.120001,"wrap failed");
  // Nonphysical/nonfinite intervals must fail closed.
  check(!Kernel(.5,0,.01,.12).evaluate(a.data(),b.data(),r),"zero dt not rejected");
  check(!Kernel(.5,-.01,.01,.12).evaluate(a.data(),b.data(),r),"negative dt not rejected");
  check(!Kernel(NAN,.02,.01,.12).evaluate(a.data(),b.data(),r),"nan rate not rejected");
  std::cout << "PASS gyro SO2 increment: " << jac_checks << " finite-difference checks, wrap and guards\n";
}
