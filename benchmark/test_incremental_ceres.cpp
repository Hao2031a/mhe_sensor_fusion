// Standalone Ceres graph lifecycle regression (no ROS).
// Compare a persistent graph that receives append / merge / prune operations
// with a freshly reconstructed reference graph on identical factor values.
#include <ceres/ceres.h>
#include <array>
#include <cmath>
#include <cstdio>
#include <deque>
#include <memory>
#include <stdexcept>
#include <vector>

constexpr int D = 9;
using X = std::array<double, D>;
using Id = ceres::ResidualBlockId;
struct Unary {
  X target;
  explicit Unary(const X & t) : target(t) {}
  template<class T> bool operator()(const T* const x, T* r) const {
    for(int j=0;j<D;++j)r[j]=(x[j]-T(target[j]))/T(0.1);
    return true;
  }
};
struct Link {
  X delta;
  explicit Link(const X & d) : delta(d) {}
  template<class T> bool operator()(const T* const a,const T* const b,T* r) const {
    for(int j=0;j<D;++j)r[j]=(b[j]-a[j]-T(delta[j]))/T(0.2);
    return true;
  }
};
static ceres::CostFunction* unary(const X & z) {
  return new ceres::AutoDiffCostFunction<Unary,D,D>(new Unary(z));
}
static ceres::CostFunction* link(const X & z) {
  return new ceres::AutoDiffCostFunction<Link,D,D,D>(new Link(z));
}
static void solve(ceres::Problem & p) {
  ceres::Solver::Options opt;
  opt.linear_solver_type=ceres::DENSE_QR;
  opt.num_threads=1;
  opt.max_num_iterations=5;
  opt.minimizer_progress_to_stdout=false;
  ceres::Solver::Summary summary;
  ceres::Solve(opt,&p,&summary);
  if(!summary.IsSolutionUsable()) throw std::runtime_error("Ceres solution failure");
}
int main() {
  ceres::Problem::Options options;
  options.enable_fast_removal=true;
  ceres::Problem incremental(options);
  std::deque<X> xs;
  std::deque<X> observations;
  std::deque<X> transitions;
  std::deque<Id> loc_ids;
  std::deque<Id> link_ids;
  X origin{};
  Id prior_id=nullptr;
  for(int cycle=0;cycle<48;++cycle) {
    X v{},target{},delta{};
    for(int j=0;j<D;++j){
      v[j]=0.002*cycle*(j+1);
      target[j]=std::sin((cycle+1)*0.02*(j+1))*0.1;
      delta[j]=0.002*(j+1);
    }
    xs.push_back(v);
    observations.push_back(target);
    transitions.push_back(delta);
    incremental.AddParameterBlock(xs.back().data(),D);
    if(xs.size()==1)prior_id=incremental.AddResidualBlock(unary(origin),nullptr,xs.front().data());
    loc_ids.push_back(incremental.AddResidualBlock(unary(target),nullptr,xs.back().data()));
    if(xs.size()>1) {
      link_ids.push_back(incremental.AddResidualBlock(link(delta),nullptr,
        xs[xs.size()-2].data(),xs.back().data()));
    } else link_ids.push_back(nullptr);
    // Near-synchronous IMU update of the latest state: replace only its local
    // factor instead of reconstructing older factors.
    if(cycle%3==0) {
      observations.back()[0]+=0.025;
      incremental.RemoveResidualBlock(loc_ids.back());
      loc_ids.back()=incremental.AddResidualBlock(
        unary(observations.back()),nullptr,xs.back().data());
    }
    if(xs.size()>8) {
      incremental.RemoveResidualBlock(prior_id);
      incremental.RemoveResidualBlock(loc_ids.front());
      incremental.RemoveResidualBlock(link_ids[1]); // x[0] -> x[1]
      incremental.RemoveParameterBlock(xs.front().data());
      xs.pop_front(); observations.pop_front(); transitions.pop_front();
      loc_ids.pop_front(); link_ids.pop_front();
      link_ids.front()=nullptr;
      // Same unary boundary prior supplied to both solvers; tests lifecycle,
      // not the Schur mathematics (covered by test_block_schur).
      origin=xs.front();
      prior_id=incremental.AddResidualBlock(unary(origin),nullptr,xs.front().data());
    }
    // Reference: fresh graph with matching factor set and initial values.
    std::deque<X> reference=xs;
    ceres::Problem full;
    for(auto & x:reference)full.AddParameterBlock(x.data(),D);
    full.AddResidualBlock(unary(origin),nullptr,reference.front().data());
    for(size_t k=0;k<reference.size();++k) {
      full.AddResidualBlock(unary(observations[k]),nullptr,reference[k].data());
      if(k) full.AddResidualBlock(link(transitions[k]),nullptr,
        reference[k-1].data(), reference[k].data());
    }
    solve(incremental);
    solve(full);
    for(size_t k=0;k<reference.size();++k)
      for(int j=0;j<D;++j)
        if(std::abs(xs[k][j]-reference[k][j])>1e-8) {
          std::fprintf(stderr,"mismatch cycle=%d k=%zu j=%d inc=%g ref=%g\n",
            cycle,k,j,xs[k][j],reference[k][j]);
          return 1;
        }
  }
  std::puts("PASS 48 incremental Ceres append/merge/prune vs reconstructed graph cycles");
}
