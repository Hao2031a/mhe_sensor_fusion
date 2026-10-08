#pragma once
// Full-horizon wheel/gyro Jacobian; shared slow [bg, sc, sd] modes.
// v_k,w_k are independent nuisance states, coupled by the same random-walk
// process as MHE. No prior/arrival residual contributes to this information.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>
namespace mhe_sensor_fusion { namespace horizon_observability {
struct Sample {
 double time_sec{0},v{0},w{0},slip_left{0},slip_right{0};
 bool wheel{false},gyro{false};
 double wheel_sigma_left{.025},wheel_sigma_right{.025},gyro_sigma{.01};
 double sigma_process_v{.1},sigma_process_w{.22};
};
struct Config {
 double wheel_separation{.3},bias_scale{.03},slip_scale{.02};
 double information_reference{.25},process_weight_scale{1.};
 double min_span_sec{.08};
 int min_wheel_samples{5},min_gyro_samples{5};
};
struct Result {
 bool valid{false};
 std::size_t states{0},wheel_samples{0},gyro_samples{0};
 int rank{0};double span_sec{0};
 std::array<double,3> conditional_information{{0,0,0}};
 std::array<double,3> scores{{0,0,0}};
 std::array<double,9> information{{0,0,0,0,0,0,0,0,0}};
};
using M2=std::array<double,4>;using M23=std::array<double,6>;using M3=std::array<double,9>;
inline double& a2(M2& m,int i,int j){return m[2*i+j];}
inline double a2(const M2& m,int i,int j){return m[2*i+j];}
inline double& a23(M23& m,int i,int j){return m[3*i+j];}
inline double a23(const M23& m,int i,int j){return m[3*i+j];}
inline double& a3(M3& m,int i,int j){return m[3*i+j];}
inline double a3(const M3& m,int i,int j){return m[3*i+j];}
inline bool invertSPD(const M2& a,M2& inv){
 const double scale=std::max({std::abs(a[0]),std::abs(a[1]),std::abs(a[2]),std::abs(a[3])});
 if(!std::isfinite(scale)||scale<=0)return false;
 const double det=(a[0]/scale)*(a[3]/scale)-(a[1]/scale)*(a[2]/scale);
 if(!(a[0]>0&&a[3]>0&&det>1e-13))return false;
 const double d=scale*det;
 inv={{(a[3]/scale)/d,-(a[1]/scale)/d,-(a[2]/scale)/d,(a[0]/scale)/d}};
 return true;
}
inline void addFactor(M2& diag,M23& cross,M3& f,
 const std::array<double,2>& a,const std::array<double,3>& b){
 for(int i=0;i<2;++i)for(int j=0;j<2;++j)a2(diag,i,j)+=a[i]*a[j];
 for(int i=0;i<2;++i)for(int j=0;j<3;++j)a23(cross,i,j)+=a[i]*b[j];
 for(int i=0;i<3;++i)for(int j=0;j<3;++j)a3(f,i,j)+=b[i]*b[j];
}
// Profile the other TWO calibration modes. Rank-one case uses the exact 2x2
// Moore-Penrose inverse, avoiding information from fictitious eigenvalue floors.
inline double conditional(const M3& f,int target){
 const int k=(target+1)%3,l=(target+2)%3;
 const double a=std::max(0.,a3(f,k,k)),b=.5*(a3(f,k,l)+a3(f,l,k));
 const double c=std::max(0.,a3(f,l,l)),tr=a+c,det=a*c-b*b;
 const double x=a3(f,k,target),y=a3(f,l,target);
 double removed=0;
 if(tr>1e-13 && std::isfinite(tr)){
  if(det>1e-10*tr*tr)removed=(c*x*x-2*b*x*y+a*y*y)/det;
  else removed=(a*x*x+2*b*x*y+c*y*y)/(tr*tr);
 }
 return std::max(0.,a3(f,target,target)-removed);
}
inline int effectiveRank(M3 m){
 for(int sweep=0;sweep<25;++sweep){
  int p=0,q=1;double off=std::abs(a3(m,0,1));
  for(const auto ij:{std::array<int,2>{{0,2}},std::array<int,2>{{1,2}}}){
   const double v=std::abs(a3(m,ij[0],ij[1]));
   if(v>off){p=ij[0];q=ij[1];off=v;}
  }
  if(off<1e-13)break;
  const double tau=(a3(m,q,q)-a3(m,p,p))/(2*a3(m,p,q));
  const double t=std::copysign(1.,tau)/(std::abs(tau)+std::sqrt(1+tau*tau));
  const double c=1/std::sqrt(1+t*t),s=t*c,app=a3(m,p,p),aqq=a3(m,q,q),apq=a3(m,p,q);
  a3(m,p,p)=app-t*apq;a3(m,q,q)=aqq+t*apq;a3(m,p,q)=a3(m,q,p)=0;
  for(int k=0;k<3;++k)if(k!=p&&k!=q){
   const double u=a3(m,k,p),v=a3(m,k,q);
   a3(m,k,p)=a3(m,p,k)=c*u-s*v;
   a3(m,k,q)=a3(m,q,k)=s*u+c*v;
  }
 }
 const double mx=std::max({0.,m[0],m[4],m[8]});
 int rank=0;
 for(int k=0;k<3;++k)if(a3(m,k,k)>std::max(1e-10,mx*1e-7))++rank;
 return rank;
}
inline Result evaluate(const std::vector<Sample>& samples,const Config& cfg){
 Result out;const std::size_t n=samples.size();out.states=n;
 if(n<2||n>256||!(cfg.wheel_separation>0&&cfg.bias_scale>0&&cfg.slip_scale>0&&
  cfg.information_reference>0&&cfg.process_weight_scale>=0)||!std::isfinite(cfg.process_weight_scale))return out;
 out.span_sec=samples.back().time_sec-samples.front().time_sec;
 if(!std::isfinite(out.span_sec)||out.span_sec<cfg.min_span_sec)return out;
 std::vector<M2> diag(n,M2{{0,0,0,0}}),edge(n,M2{{0,0,0,0}});
 std::vector<M23> cross(n,M23{{0,0,0,0,0,0}});
 M3 F{{0,0,0,0,0,0,0,0,0}};const double track=.5*cfg.wheel_separation;
 for(std::size_t k=0;k<n;++k){
  const auto& s=samples[k];
  if(!std::isfinite(s.v)||!std::isfinite(s.w)||!std::isfinite(s.slip_left)||!std::isfinite(s.slip_right))return out;
  if(k>0){
   const double dt=s.time_sec-samples[k-1].time_sec;
   if(!(dt>0)||!std::isfinite(dt))return out;
   const double pv=cfg.process_weight_scale/std::pow(std::max(s.sigma_process_v,1e-6),2);
   const double pw=cfg.process_weight_scale/std::pow(std::max(s.sigma_process_w,1e-6),2);
   if(!std::isfinite(pv)||!std::isfinite(pw))return out;
   a2(diag[k-1],0,0)+=pv;a2(diag[k],0,0)+=pv;a2(edge[k],0,0)=-pv;
   a2(diag[k-1],1,1)+=pw;a2(diag[k],1,1)+=pw;a2(edge[k],1,1)=-pw;
  }
  if(s.wheel){
   if(!(s.wheel_sigma_left>0&&s.wheel_sigma_right>0))return out;
   ++out.wheel_samples;
   const double gl=(s.v-track*s.w)/s.wheel_sigma_left,gr=(s.v+track*s.w)/s.wheel_sigma_right;
   addFactor(diag[k],cross[k],F,
    {{(1+s.slip_left)/s.wheel_sigma_left,-track*(1+s.slip_left)/s.wheel_sigma_left}},
    {{0,gl*cfg.slip_scale,-gl*cfg.slip_scale}});
   addFactor(diag[k],cross[k],F,
    {{(1+s.slip_right)/s.wheel_sigma_right,track*(1+s.slip_right)/s.wheel_sigma_right}},
    {{0,gr*cfg.slip_scale,gr*cfg.slip_scale}});
  }
  if(s.gyro){
   if(!(s.gyro_sigma>0))return out;
   ++out.gyro_samples;
   addFactor(diag[k],cross[k],F,{{0,1/s.gyro_sigma}},
    {{cfg.bias_scale/s.gyro_sigma,0,0}});
  }
 }
 if(out.wheel_samples<static_cast<std::size_t>(std::max(1,cfg.min_wheel_samples))||
    out.gyro_samples<static_cast<std::size_t>(std::max(1,cfg.min_gyro_samples)))return out;
 // O(N) block Schur against per-timestamp (v,w).
 for(std::size_t k=0;k<n;++k){
  M2 inv;if(!invertSPD(diag[k],inv))return out;
  M23 solved{{0,0,0,0,0,0}};
  for(int i=0;i<2;++i)for(int j=0;j<3;++j)
   a23(solved,i,j)=a2(inv,i,0)*a23(cross[k],0,j)+a2(inv,i,1)*a23(cross[k],1,j);
  for(int i=0;i<3;++i)for(int j=0;j<3;++j)
   a3(F,i,j)-=a23(cross[k],0,i)*a23(solved,0,j)+a23(cross[k],1,i)*a23(solved,1,j);
  if(k+1<n){
   const M2& e=edge[k+1];M2 einv{{0,0,0,0}};
   for(int i=0;i<2;++i)for(int j=0;j<2;++j)
    a2(einv,i,j)=a2(e,i,0)*a2(inv,0,j)+a2(e,i,1)*a2(inv,1,j);
   for(int i=0;i<2;++i)for(int j=0;j<2;++j)
    a2(diag[k+1],i,j)-=a2(einv,i,0)*a2(e,0,j)+a2(einv,i,1)*a2(e,1,j);
   for(int i=0;i<2;++i)for(int j=0;j<3;++j)
    a23(cross[k+1],i,j)-=a2(e,i,0)*a23(solved,0,j)+a2(e,i,1)*a23(solved,1,j);
  }
 }
 for(int i=0;i<3;++i)for(int j=i+1;j<3;++j){
  const double x=.5*(a3(F,i,j)+a3(F,j,i));a3(F,i,j)=a3(F,j,i)=x;
 }
 const double scale=std::max({1.,std::abs(F[0]),std::abs(F[4]),std::abs(F[8])});
 for(int i=0;i<3;++i){
  if(!std::isfinite(a3(F,i,i))||a3(F,i,i)<-1e-8*scale)return out;
  a3(F,i,i)=std::max(0.,a3(F,i,i));
 }
 for(double v:F)if(!std::isfinite(v))return out;
 out.rank=effectiveRank(F);
 for(int i=0;i<3;++i){
  out.conditional_information[i]=conditional(F,i);
  out.scores[i]=std::clamp(1-std::exp(-out.conditional_information[i]/cfg.information_reference),0.,1.);
 }
 out.information=F;out.valid=true;return out;
}
}} // namespace mhe_sensor_fusion::horizon_observability
