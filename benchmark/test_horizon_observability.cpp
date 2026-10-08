#include "mhe_sensor_fusion/horizon_observability.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>
namespace ob=mhe_sensor_fusion::horizon_observability;
static std::vector<ob::Sample> scenario(int mode){
 std::vector<ob::Sample> out;
 for(int k=0;k<25;++k){
  ob::Sample s;s.time_sec=.01*k;
  s.v=.25+((mode==1||mode==3)?.14*std::sin(.27*k):0.);
  s.w=.5+((mode==2||mode==3)?.7*std::cos(.39*k):0.);
  if(mode==4){s.v=.25+.14*std::sin(.2*k);s.w=.5+.7*std::sin(.2*k);}
  s.gyro=true;s.wheel=true;
  s.sigma_process_v=.08;s.sigma_process_w=.12;
  out.push_back(s);
 }
 return out;
}
int main(){
 ob::Config cfg;
 auto r0=ob::evaluate(scenario(0),cfg);
 auto rv=ob::evaluate(scenario(1),cfg);
 auto rw=ob::evaluate(scenario(2),cfg);
 auto both=ob::evaluate(scenario(3),cfg);
 auto coupled=ob::evaluate(scenario(4),cfg);
 for(auto x:{r0,rv,rw,both,coupled})assert(x.valid);
 for(double s:r0.scores)assert(s<1e-5);
 assert(rv.scores[2]>r0.scores[2]+1e-5);
 assert(rw.scores[1]>r0.scores[1]+1e-5);
 assert(both.scores[1]>0 && both.scores[2]>0);
 for(double s:coupled.scores)assert(s<.02);
 {auto bad=scenario(3);bad[10].time_sec=bad[9].time_sec;assert(!ob::evaluate(bad,cfg).valid);}
 {auto shortw=scenario(3);shortw.resize(2);assert(!ob::evaluate(shortw,cfg).valid);}
 {auto bad=scenario(3);bad[2].wheel_sigma_left=0;assert(!ob::evaluate(bad,cfg).valid);}
 {auto s=scenario(0);cfg.process_weight_scale=0;auto r=ob::evaluate(s,cfg);assert(r.valid);
  for(double x:r.scores)assert(x<1e-5);}
 cfg.process_weight_scale=1;
 std::mt19937 rng(20261008);std::uniform_real_distribution<double> u(-1,1);
 int passes=0;
 for(int trial=0;trial<1000;++trial){
  auto ss=scenario(3);
  for(auto& s:ss){s.v+=.10*u(rng);s.w+=.4*u(rng);
   s.gyro=(trial%7!=0||&s==&ss[0]);s.wheel=(trial%9!=0||&s==&ss[0]);}
  auto r=ob::evaluate(ss,cfg);
  if(r.valid){for(double x:r.scores)assert(std::isfinite(x)&&x>=0&&x<=1);++passes;}
 }
 std::cout<<"PASS full-horizon nuisance-profiled observability, randomized cases="<<passes<<"\n";
}
