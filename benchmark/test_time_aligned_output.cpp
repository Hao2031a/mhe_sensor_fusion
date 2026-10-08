#include "mhe_sensor_fusion/time_aligned_output.hpp"
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
  using namespace mhe_sensor_fusion::time_align;
  const double max_gap = 0.05;
  const double timeout = 0.20;
  auto p = planStep(false, 0.01, true, 0.01, max_gap, timeout);
  assert(p.status == StepStatus::FirstTick && p.stop_twist && p.dt_sec == 0.0);
  p = planStep(true, 0.01, true, 0.01818, max_gap, timeout);
  assert(p.status == StepStatus::Advance && !p.stop_twist);
  assert(std::abs(p.dt_sec - 0.01) < 1e-12);
  p = planStep(true, 0.03, true, 0.01818, max_gap, timeout);
  assert(p.status == StepStatus::Advance && std::abs(p.dt_sec - 0.03) < 1e-12);
  p = planStep(true, 0.0, true, 0.01, max_gap, timeout);
  assert(p.status == StepStatus::Paused && p.stop_twist);
  p = planStep(true, -0.2, true, 0.01, max_gap, timeout);
  assert(p.status == StepStatus::BackwardJump && p.stop_twist);
  p = planStep(true, 0.100, true, 0.02, max_gap, timeout);
  assert(p.status == StepStatus::ForwardGap && p.stop_twist && p.dt_sec == 0.0);
  p = planStep(true, 0.01, true, 0.25, max_gap, timeout);
  assert(p.status == StepStatus::StaleMeasurement && p.stop_twist);
  p = planStep(true, 0.01, true, -0.04, max_gap, timeout);
  assert(p.status == StepStatus::FutureMeasurement && p.stop_twist);
  p = planStep(true, 0.01, false, 0.0, max_gap, timeout);
  assert(p.status == StepStatus::StaleMeasurement && p.stop_twist);
  p = planStep(true, std::numeric_limits<double>::quiet_NaN(), true,
               0.01, max_gap, timeout);
  assert(p.stop_twist);
  assert(eventIsDue(1000000000, 1000000000, 2000000));
  assert(eventIsDue(1001000000, 1000000000, 2000000));
  assert(!eventIsDue(1010000000, 1000000000, 2000000));
  assert(eventIsDue(1000000001, 1000000000, 0) == false);
  std::cout << "PASS: simulation timestamp, pause, future, stale and jump guards\n";
}
