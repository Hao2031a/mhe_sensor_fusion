#include "mhe_sensor_fusion/se2_error_correction.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <random>
using namespace mhe_fusion::se2_correct;
static void check(bool good,const char* message){ if(!good)throw std::runtime_error(message); }
int main(){
  int cases=0; Policy p;p.enabled=true;Pose pub{1,2,3.13},ref{1,2,-3.13};
  auto a=reconcile(pub,ref,0.01,0.01,0.,true,p);
  check(a.applied && a.yaw_error>0 && a.yaw_error<0.03,"SO2 wrap");++cases;
  check(std::abs(a.applied_yaw)<=p.max_angular_rate*0.01+1e-12,"yaw cap");++cases;
  check(!reconcile(pub,ref,0.01,0.1,0.,true,p).applied,"stale gate");++cases;
  check(!reconcile(pub,ref,0.01,0.01,0.005,true,p).applied,"skew gate");++cases;
  check(!reconcile(pub,ref,0.01,0.01,0,false,p).applied,"trust gate");++cases;
  p.enabled=false;
  check(!reconcile(pub,ref,0.01,0.01,0,true,p).applied,"disabled by default");++cases;
  p.enabled=true; p.max_error_m=0.2; p.max_linear_rate=0.04;
  std::mt19937_64 gen(73);std::uniform_real_distribution<double> u(-1.0,1.0);
  for(int i=0;i<3000;++i){
    pub={u(gen),u(gen),3.14*u(gen)};
    ref={pub.x+0.05*u(gen),pub.y+0.05*u(gen),pub.yaw+0.1*u(gen)};
    a=reconcile(pub,ref,0.01,0.015,0.,true,p);
    check(a.applied_distance <= 0.0004+1e-12,"translation rate cap");
    check(std::abs(a.applied_yaw) <=0.0012+1e-12,"rotation rate cap");
    check(std::isfinite(a.pose.x) && std::isfinite(a.pose.y),"finite SE2 pose");
    cases+=3;
  }
  // Exact timestamp prediction (a right-angle turn arc at 1rad/s).
  Pose start{0,0,0};auto pred=predict(start,1,1,1.);
  check(std::abs(pred.x-std::sin(1.))<1e-12 &&
        std::abs(pred.y-(1-std::cos(1.)))<1e-12,"exact arc reference");++cases;
  std::cout<<"PASS: "<<cases<<" SE2 correction checks\n";
}
