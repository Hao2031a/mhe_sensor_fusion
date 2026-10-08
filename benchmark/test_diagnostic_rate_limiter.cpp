#include "mhe_sensor_fusion/diagnostic_rate_limiter.hpp"
#include <cassert>
#include <cstdint>
#include <iostream>
int main()
{
  mhe_sensor_fusion::DiagnosticRateLimiter limiter;
  int due = 0;
  // 10s of exact 100-Hz ROS ticks, 25-Hz diagnostic target.
  for (std::int64_t k = 0; k < 1000; ++k) {
    due += static_cast<int>(limiter.due(k * 10000000, 25.0));
  }
  assert(due == 250);
  assert(limiter.due(0, 25.0));  // Gazebo /clock rewind.
  assert(!limiter.due(10000000, 25.0));
  limiter.reset();
  assert(limiter.due(0, 25.0));
  std::cout << "DIAGNOSTIC_RATE_TEST_PASS 1000 ticks / 250 publications\n";
}
