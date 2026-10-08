#include "mhe_sensor_fusion/rt_budget.hpp"
#include <cmath>
#include <cstdlib>
#include <iostream>

using mhe_fusion::rt::CovarianceDecision;
using mhe_fusion::rt::CovarianceSchedule;
using mhe_fusion::rt::covarianceDecision;

int main()
{
  int checks = 0;
  auto verify = [&](bool good, const char * what) {
      if (!good) {
        std::cerr << "FAIL: " << what << '\n';
        std::exit(1);
      }
      ++checks;
    };
  CovarianceSchedule cfg;
  verify(covarianceDecision(cfg, 1, 1, true, 1.0, 3.0) == CovarianceDecision::NotDue,
    "not due");
  verify(covarianceDecision(cfg, 20, 20, true, 3.0, 3.0) == CovarianceDecision::Compute,
    "within budget");
  verify(covarianceDecision(cfg, 20, 20, true, 7.0, 3.0) == CovarianceDecision::Defer,
    "defer when predicted over budget");
  verify(covarianceDecision(cfg, 21, 21, false, 7.0, 3.0) == CovarianceDecision::Compute,
    "initial covariance required");
  verify(covarianceDecision(cfg, 41, 60, true, 7.0, 3.0) == CovarianceDecision::Compute,
    "maximum age forces eventual refresh");
  verify(covarianceDecision(cfg, 20, 20, true, NAN, 1.0) == CovarianceDecision::Defer,
    "nonfinite timing safe");
  verify(covarianceDecision(cfg, 20, 20, true, 7.0, NAN) == CovarianceDecision::Defer,
    "nonfinite covariance estimate uses reserve");
  verify(mhe_fusion::rt::deadlineMiss(11.0, 10.0), "deadline overrun");
  verify(!mhe_fusion::rt::deadlineMiss(10.0, 10.0), "exact deadline is on time");
  verify(!mhe_fusion::rt::deadlineMiss(NAN, 10.0), "nonfinite duration not counted");
  cfg.defer_on_overrun = false;
  verify(covarianceDecision(cfg, 20, 20, true, 30.0, 3.0) == CovarianceDecision::Compute,
    "optional unthrottled covariance");
  verify(mhe_fusion::rt::canSnapshot(2., 1., 10., 4.5, 2.5, 1.0),
    "snapshot allowed within total callback deadline");
  verify(!mhe_fusion::rt::canSnapshot(7., 1., 10., 4.5, 2.5, 1.0),
    "snapshot rejected when remaining tick insufficient");
  verify(!mhe_fusion::rt::canSnapshot(2., 5., 10., 4.5, 2.5, 1.0),
    "snapshot rejected when solve too slow even if covariance stale");
  verify(!mhe_fusion::rt::canSnapshot(NAN, 1., 10., 4.5, 2.5, 1.0),
    "snapshot reject nonfinite scheduling input");
  std::cout << "PASS: " << checks << " real-time budget policy tests\n";
  return 0;
}
