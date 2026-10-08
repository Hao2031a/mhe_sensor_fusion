#include "mhe_sensor_fusion/stability_guards.hpp"
#include "mhe_sensor_fusion/solver_fallback_policy.hpp"
#include <cmath>
#include <iostream>
#include <limits>

using namespace mhe_fusion::safety;

int main()
{
  // Explicit checks: assert() is compiled out under -DNDEBUG/Release.
  int failed = 0;
  int total = 0;
  auto check = [&](bool cond, const char * name) {
      ++total;
      if (!cond) {
        ++failed;
        std::cerr << "FAIL: " << name << '\n';
      }
    };
  const auto normal = classifyInnovation(1.0, 9.0, 64.0, 3.0);
  const auto soft = classifyInnovation(36.0, 9.0, 64.0, 3.0);
  const auto hard = classifyInnovation(100.0, 9.0, 64.0, 3.0);
  check(normal.state == GateState::Normal, "normal gate");
  check(normal.sigma_scale == 1.0, "normal sigma");
  check(soft.state == GateState::Soft, "soft gate");
  check(std::abs(soft.sigma_scale - 2.0) < 1e-12, "soft sigma");
  check(hard.state == GateState::Hard, "hard gate");
  check(classifyInnovation(std::numeric_limits<double>::quiet_NaN(), 9.0, 64.0, 3.0).state == GateState::Hard, "nonfinite gate");
  check(costAcceptable(10.0, 9.0), "decreasing cost");
  check(!costAcceptable(10.0, 12.0), "increasing cost rejection");
  check(!costAcceptable(std::numeric_limits<double>::infinity(), 9.0), "infinite cost rejection");
  check(endpointJumpAcceptable(0.1, 0.1, 0.0, 0.0, 0.01, 8.0, 30.0, 0.35, 1.0), "allowed endpoint jump");
  check(!endpointJumpAcceptable(1.0, 0.0, 0.0, 0.0, 0.01, 8.0, 30.0, 0.35, 1.0), "linear jump rejection");
  check(!endpointJumpAcceptable(0.0, 3.0, 0.0, 0.0, 0.01, 8.0, 30.0, 0.35, 1.0), "angular jump rejection");
  check(classifySolveResult(true, true, true, true) == SolveRejectReason::None,
        "accepted solution classification");
  check(classifySolveResult(false, true, true, true) == SolveRejectReason::Numerical,
        "numerical failure classification");
  check(classifySolveResult(true, false, true, true) == SolveRejectReason::Numerical,
        "nonfinite solution classification");
  check(classifySolveResult(true, true, false, true) == SolveRejectReason::Cost,
        "cost failure classification");
  check(classifySolveResult(true, true, true, false) == SolveRejectReason::Endpoint,
        "endpoint failure classification");

  auto qr = planQrFallback(SolveRejectReason::Numerical, false, 2.0, 6.0, 1.0);
  check(qr.run && std::abs(qr.remaining_ms - 4.0) < 1e-12,
        "numerical failure QR gets remaining budget only");
  check(!planQrFallback(SolveRejectReason::Endpoint, false, 2.0, 6.0, 1.0).run,
        "physical endpoint rejection does not trigger QR");
  check(planQrFallback(SolveRejectReason::Endpoint, false, 2.0, 6.0, 1.0, true).run,
        "endpoint retry opt-in works");
  check(!planQrFallback(SolveRejectReason::Cost, false, 5.5, 6.0, 1.0).run,
        "skip QR if minimum budget unavailable");
  check(!planQrFallback(SolveRejectReason::Cost, false, 7.0, 6.0, 1.0).run,
        "skip QR after first solve already overran budget");
  check(!planQrFallback(SolveRejectReason::Numerical, true, 0.5, 6.0, 1.0).run,
        "never fallback when first solve used QR");
  check(!planQrFallback(SolveRejectReason::None, false, 0.5, 6.0, 1.0).run,
        "no retry on acceptable result");
  check(!planQrFallback(SolveRejectReason::Numerical, false,
                        std::numeric_limits<double>::quiet_NaN(), 6.0, 1.0).run,
        "reject invalid fallback timing");
  check(!planQrFallback(SolveRejectReason::Numerical, false, 0.0, -1.0, 1.0).run,
        "reject invalid budget");
  if (failed) {
    std::cerr << "FAIL: " << failed << '/' << total << " checks\n";
    return 1;
  }
  std::cout << "PASS: " << total << " C++ stability guard checks\n";
  return 0;
}
