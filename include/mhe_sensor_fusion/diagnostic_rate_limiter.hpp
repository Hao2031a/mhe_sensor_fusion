#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace mhe_sensor_fusion
{
// ROS-time based, allocation-free limiter for best-effort informational topics.
// It never gates /odom, TF or real-time profiles.
class DiagnosticRateLimiter
{
public:
  bool due(std::int64_t stamp_ns, double frequency_hz)
  {
    if (!initialized_ || stamp_ns < last_ns_) {
      initialized_ = true;
      last_ns_ = stamp_ns;
      return true;
    }
    const double interval_ns = 1.0e9 / frequency_hz;
    if (static_cast<double>(stamp_ns - last_ns_) + 1.0 >= interval_ns) {
      last_ns_ = stamp_ns;
      return true;
    }
    return false;
  }

  void reset() noexcept
  {
    initialized_ = false;
    last_ns_ = 0;
  }

private:
  bool initialized_{false};
  std::int64_t last_ns_{0};
};
}  // namespace mhe_sensor_fusion
