#include "mhe_sensor_fusion/low_latency_output.hpp"
// Enable runtime assertions even under CMake Release (-DNDEBUG).
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

int main()
{
  using mhe_sensor_fusion::YawFeedbackPolicy;
  using mhe_sensor_fusion::boundedYawFeedback;
  using mhe_sensor_fusion::wrapYaw;
  YawFeedbackPolicy p{};
  auto r = boundedYawFeedback(0.0, 0.1, 0.0, 0.01, 0.01, p);
  assert(!r.applied && r.step_rad == 0.0);  // opt-in only

  // Solver endpoint must be accepted for exactly the sensor epoch being used.
  assert(!mhe_sensor_fusion::trustworthyYawEndpoint(
    false, 0.0, 1.0, 0.0, false, 0.0, 0, p));
  assert(!mhe_sensor_fusion::trustworthyYawEndpoint(
    true, 0.010, 1.0, 0.0, false, 0.0, 0, p));
  assert(!mhe_sensor_fusion::trustworthyYawEndpoint(
    true, 0.0, 0.10, 0.0, false, 0.0, 0, p));
  assert(!mhe_sensor_fusion::trustworthyYawEndpoint(
    true, 0.0, 1.0, 30.0, false, 0.0, 0, p));
  assert(mhe_sensor_fusion::trustworthyYawEndpoint(
    true, 0.0, 1.0, 0.0, false, 0.0, 0, p));
  p.require_covariance = true;
  assert(!mhe_sensor_fusion::trustworthyYawEndpoint(
    true, 0.0, 1.0, 0.0, false, 0.0, 0, p));
  assert(!mhe_sensor_fusion::trustworthyYawEndpoint(
    true, 0.0, 1.0, 0.0, true, 0.50, 0, p));
  assert(mhe_sensor_fusion::trustworthyYawEndpoint(
    true, 0.0, 1.0, 0.0, true, 0.001, 0, p));
  p.require_covariance = false;
  p.enabled = true;
  r = boundedYawFeedback(0.0, 0.1, 0.0, 0.01, 0.01, p);
  assert(r.applied);
  assert(r.step_rad > 0.0 && r.step_rad <= 0.002000001);

  // Endpoint at 1.0 rad, measured 20 ms ago, turning at 2 rad/s.
  // A pose propagated to *now* at 1.04 rad should need no correction.
  r = boundedYawFeedback(1.04, 1.00, 2.0, 0.02, 0.01, p);
  assert(!r.applied);
  assert(std::abs(r.error_rad) < 1e-12);

  // Crossing the [-pi, pi] boundary must choose the short arc.
  r = boundedYawFeedback(-3.13, 3.13, 0.0, 0.0, 0.01, p);
  assert(r.applied && r.step_rad < 0.0);
  assert(std::abs(r.error_rad) < 0.03);

  // A stale or future endpoint, an outlier, invalid or excessive dt must
  // never change yaw. These are key protections under Gazebo pause/lag.
  for (double age : {-0.001, 0.041, 1.0}) {
    r = boundedYawFeedback(0.0, 0.1, 0.0, age, 0.01, p);
    assert(!r.applied);
  }
  r = boundedYawFeedback(0.0, 1.0, 0.0, 0.01, 0.01, p);
  assert(!r.applied);
  r = boundedYawFeedback(0.0, 0.1, 0.0, 0.01, 0.2, p);
  assert(!r.applied);
  r = boundedYawFeedback(0.0, std::numeric_limits<double>::quiet_NaN(), 0.0, 0.01, 0.01, p);
  assert(!r.applied);

  // Rate limit survives both negative and positive error sequences.
  for (int k = -100; k <= 100; ++k) {
    const double err = k * 0.001;
    r = boundedYawFeedback(0.0, err, 0.0, 0.01, 0.01, p);
    assert(std::abs(r.step_rad) <= p.max_rate_rad_s * 0.01 + 1e-12);
    if (r.applied) {
      assert(r.step_rad * err > 0.0);
    }
  }
  assert(std::abs(wrapYaw(4 * mhe_sensor_fusion::kPi)) < 1e-12);
  std::cout << "PASS: low-latency yaw feedback gates and rate limits" << std::endl;
}
