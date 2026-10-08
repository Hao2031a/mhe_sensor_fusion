#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace mhe_sensor_fusion
{
namespace time_align
{

enum class StepStatus : int
{
  Advance = 0, FirstTick = 1, Paused = 2, BackwardJump = 3,
  ForwardGap = 4, FutureMeasurement = 5, StaleMeasurement = 6
};

struct Step
{
  double dt_sec{0.0};
  double sensor_age_sec{0.0};
  StepStatus status{StepStatus::FirstTick};
  bool stop_twist{true};
};

// This function deliberately never *clamps* dt while advertising the original
// publication timestamp. A large clock discontinuity holds pose and twist,
// preventing an under-integrated pose from being stamped as current.
inline Step planStep(
  bool have_last_tick, double delta_sec, bool have_measurement,
  double age_sec, double max_gap_sec, double sensor_timeout_sec,
  double future_tolerance_sec = 0.002) noexcept
{
  Step result;
  if (!have_last_tick) {
    result.status = StepStatus::FirstTick;
    return result;
  }
  if (!std::isfinite(delta_sec) || delta_sec < -1e-6) {
    result.status = StepStatus::BackwardJump;
    return result;
  }
  if (delta_sec <= 1e-6) {
    result.status = StepStatus::Paused;
    return result;
  }
  if (delta_sec > std::max(0.001, max_gap_sec)) {
    result.status = StepStatus::ForwardGap;
    return result;
  }
  if (!have_measurement || !std::isfinite(age_sec)) {
    result.status = StepStatus::StaleMeasurement;
    return result;
  }
  result.sensor_age_sec = age_sec;
  if (age_sec < -std::max(0.0, future_tolerance_sec)) {
    result.status = StepStatus::FutureMeasurement;
    return result;
  }
  if (age_sec > sensor_timeout_sec) {
    result.status = StepStatus::StaleMeasurement;
    return result;
  }
  result.dt_sec = delta_sec;
  result.status = StepStatus::Advance;
  result.stop_twist = false;
  return result;
}

inline bool eventIsDue(
  std::int64_t event_stamp_ns, std::int64_t now_ns,
  std::int64_t tolerance_ns) noexcept
{
  if (tolerance_ns < 0) {
    tolerance_ns = 0;
  }
  // Compare by subtraction to avoid integer overflow on addition.
  return event_stamp_ns <= now_ns ||
    (event_stamp_ns - now_ns) <= tolerance_ns;
}

}  // namespace time_align
}  // namespace mhe_sensor_fusion
