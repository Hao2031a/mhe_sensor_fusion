#include "mhe_sensor_fusion/horizon_observability.hpp"
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>
namespace ob=mhe_sensor_fusion::horizon_observability;
using Vec=std::vector<double>;
// Independently form the full white Jacobian and eliminate the 2N nuisance
// columns by dense Gaussian elimination with partial row pivoting.
static std::array<double,9> reference(const std::vector<ob::Sample>& ss,const ob::Config& cfg){
 const size_t n=ss.size(),dim=2*n+3;
 std::vector<Vec> rows;
 auto add=[&](size_t k,double v,double w,double bg,double sc,double sd){
  Vec x(dim,0);x[2*k]=v;x[2*k+1]=w;
  x[2*n]=bg;x[2*n+1]=sc;x[2*n+2]=sd;rows.push_back(std::move(x));
 };
 const double b=.5*cfg.wheel_separation;
 for(size_t k=0;k<n;++k){
  const auto& s=ss[k];
  if(s.wheel){
   const double l=(s.v-b*s.w)/s.wheel_sigma_left,r=(s.v+b*s.w)/s.wheel_sigma_right;
   add(k,(1+s.slip_left)/s.wheel_sigma_left,-b*(1+s.slip_left)/s.wheel_sigma_left,
       0,l*cfg.slip_scale,-l*cfg.slip_scale);
   add(k,(1+s.slip_right)/s.wheel_sigma_right,b*(1+s.slip_right)/s.wheel_sigma_right,
       0,r*cfg.slip_scale,r*cfg.slip_scale);
  }
  if(s.gyro)add(k,0,1/s.gyro_sigma,cfg.bias_scale/s.gyro_sigma,0,0);
  if(k>0){
   const double pv=std::sqrt(cfg.process_weight_scale)/s.sigma_process_v;
   const double pw=std::sqrt(cfg.process_weight_scale)/s.sigma_process_w;
   Vec v(dim,0),w(dim,0);
   v[2*k]=pv;v[2*(k-1)]=-pv;
   w[2*k+1]=pw;w[2*(k-1)+1]=-pw;
   rows.push_back(std::move(v));rows.push_back(std::move(w));
  }
 }
 Vec H(dim*dim,0);
 for(const auto& r:rows)for(size_t i=0;i<dim;++i)for(size_t j=0;j<dim;++j)
  H[i*dim+j]+=r[i]*r[j];
 const size_t m=2*n;
 Vec augmented(m*(m+3),0);
 for(size_t i=0;i<m;++i){
  for(size_t j=0;j<m;++j)augmented[i*(m+3)+j]=H[i*dim+j];
  for(size_t j=0;j<3;++j)augmented[i*(m+3)+m+j]=H[i*dim+m+j];
 }
 for(size_t k=0;k<m;++k){
  size_t pivot=k;
  for(size_t i=k+1;i<m;++i)if(std::abs(augmented[i*(m+3)+k])>
   std::abs(augmented[pivot*(m+3)+k]))pivot=i;
  assert(std::abs(augmented[pivot*(m+3)+k])>1e-12);
  if(pivot!=k)for(size_t j=0;j<m+3;++j)
   std::swap(augmented[pivot*(m+3)+j],augmented[k*(m+3)+j]);
  for(size_t i=k+1;i<m;++i){
   const double scale=augmented[i*(m+3)+k]/augmented[k*(m+3)+k];
   for(size_t j=k;j<m+3;++j)augmented[i*(m+3)+j]-=scale*augmented[k*(m+3)+j];
  }
 }
 Vec solutions(m*3,0);
 for(int i=int(m)-1;i>=0;--i)for(int j=0;j<3;++j){
  double v=augmented[size_t(i)*(m+3)+m+j];
  for(size_t k=i+1;k<m;++k)v-=augmented[size_t(i)*(m+3)+k]*solutions[3*k+j];
  solutions[3*size_t(i)+j]=v/augmented[size_t(i)*(m+3)+i];
 }
 std::array<double,9> F{};
 for(int i=0;i<3;++i)for(int j=0;j<3;++j){
  F[3*i+j]=H[(m+i)*dim+m+j];
  for(size_t k=0;k<m;++k)F[3*i+j]-=H[(m+i)*dim+k]*solutions[3*k+j];
 }
 return F;
}
int main(){
 std::mt19937_64 rng(20261008);
 std::uniform_real_distribution<double> u(-1,1);
 ob::Config cfg;cfg.min_wheel_samples=3;cfg.min_gyro_samples=3;cfg.min_span_sec=.01;
 double error=0;int cases=0;
 for(int n=4;n<=12;++n)for(int rep=0;rep<100;++rep){
  std::vector<ob::Sample> ss;
  for(int i=0;i<n;++i){
   ob::Sample s;s.time_sec=i*.012;s.v=.15+.2*u(rng);s.w=.4+u(rng);
   s.slip_left=.01*u(rng);s.slip_right=.01*u(rng);
   s.wheel=true;s.gyro=true;
   s.wheel_sigma_left=.015+.02*(u(rng)+1);
   s.wheel_sigma_right=.015+.02*(u(rng)+1);
   s.gyro_sigma=.007+.012*(u(rng)+1);
   s.sigma_process_v=.07+.07*(u(rng)+1);
   s.sigma_process_w=.14+.1*(u(rng)+1);
   ss.push_back(s);
  }
  auto estimate=ob::evaluate(ss,cfg);assert(estimate.valid);
  auto truth=reference(ss,cfg);
  for(int j=0;j<9;++j){
   const double delta=std::abs(estimate.information[j]-truth[j]);
   error=std::max(error,delta);
   assert(delta<1e-6*std::max(1.,std::abs(truth[j])));
  }
  ++cases;
 }
 std::cout<<"PASS 900 full-horizon dense Schur reference cases, max_abs_err="<<error<<"\n";
}
