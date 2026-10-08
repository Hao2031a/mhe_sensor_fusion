#include "mhe_sensor_fusion/scaled_covariance.hpp"
#include "mhe_sensor_fusion/fast_covariance.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <random>
using namespace mhe_fusion;
static void check(bool good, const char* why) { if (!good) throw std::runtime_error(why); }
int main() {
  int checks=0;
  // Normal covariance with units differing by 1e6: Jacobi scaling must
  // recover exact selected marginal and avoid false ill-conditioning.
  Eigen::MatrixXd J=Eigen::MatrixXd::Zero(4,3);
  J<<0.001, 0.0, 0.0, 0.0001, 5.0, 0.0, 0.0, 0.5, 200., 0.0, 0.0, 2.;
  auto got=scaled_cov::selected(J,2);
  Eigen::MatrixXd ref=(J.transpose()*J).inverse().bottomRightCorner(2,2);
  check(got.valid && got.method==1,"scaled LLT full rank"); ++checks;
  check((got.covariance-ref).norm()<1e-12,"scaled LLT matches reference"); ++checks;
  // Synthetic adversarial ill-conditioning: force SVD based on reciprocal
  // condition but retain full rank and compare against analytic diagonal.
  Eigen::MatrixXd poorly(2,2);
  poorly << 1.0, 1.0, 0.0, 1e-4;
  scaled_cov::Config cfg; cfg.minimum_rcond=0.1; cfg.maximum_svd_condition=1e7;
  auto fallback=scaled_cov::selected(poorly,1,cfg);
  check(fallback.valid && fallback.method==2,"SVD fallback used"); ++checks;
  check(std::abs(fallback.covariance(0,0)-1e8)<1.0,"SVD variance correct"); ++checks;
  cfg.allow_svd=false;
  check(!scaled_cov::selected(poorly,1,cfg).valid,"SVD can be disabled"); ++checks;
  // Full rank but too poorly conditioned for reliable inversion: reject.
  cfg.allow_svd=true; cfg.maximum_svd_condition=100.0;
  check(!scaled_cov::selected(poorly,1,cfg).valid,"reject extreme SVD condition"); ++checks;
  Eigen::MatrixXd rank1=Eigen::MatrixXd::Ones(4,3);
  check(!scaled_cov::selected(rank1,1).valid,"never pseudoinverse nullspace"); ++checks;
  // Random SPD cases, compare LLT vs explicitly inverted normal equations.
  std::mt19937_64 engine(20261008);
  std::normal_distribution<double> noise;
  for(int trial=0;trial<120;++trial){
    Eigen::MatrixXd A=Eigen::MatrixXd::Zero(22,9);
    for(int i=0;i<A.rows();++i) for(int j=0;j<A.cols();++j) A(i,j)=noise(engine);
    A.topRows(9).diagonal().array() += 2.0;
    scaled_cov::Config c;
    auto cov=scaled_cov::selected(A,3,c);
    check(cov.valid,"SPD trial valid");
    auto exact=(A.transpose()*A).inverse().bottomRightCorner(3,3).eval();
    check((cov.covariance-exact).norm() < 1e-9,"SPD covariance equality");
    checks+=2;
  }
  // Actual fast covariance worker CRS integration with new snapshot flags.
  fast_cov::Snapshot s;
  s.rows=2;s.cols=2;s.tail_size=2;
  s.row_offsets={0,1,2};s.column_indices={0,1};s.values={2.,4.};
  s.jacobi_scaled_enabled=true;
  const auto res=fast_cov::compute(s);
  check(res.valid && res.method==1 && std::abs(res.covariance[0]-0.25)<1e-12,
      "CRS worker dispatch"); ++checks;
  std::cout<<"PASS: "<<checks<<" scaled covariance checks\n";
}
