#pragma once

// Bounded right-invariant SE(2) correction for an odometry propagated to the
// SAME timestamp as a trusted Ceres endpoint. Only pose is corrected; twist
// remains the physical MHE estimate (never report a fake wheel/gyro rate).
#include "mhe_sensor_fusion/se2_yaw_math.hpp"
#include <algorithm>
#include <cmath>

namespace mhe_fusion::se2_correct
{
struct Pose
{
  double x{0.0}, y{0.0}, yaw{0.0};
};
struct Policy
{
  bool enabled{false};
  double gain{0.12};
  double max_linear_rate{0.03};   // meters of correction per second
  double max_angular_rate{0.12}; // radians of correction per second
  double max_error_m{0.15};
  double max_error_rad{0.20};
  double max_age_sec{0.04};
  double max_reference_skew_sec{0.002};
};
struct Step
{
  bool applied{false};
  double distance_error{0.0};
  double yaw_error{0.0};
  double applied_distance{0.0};
  double applied_yaw{0.0};
  Pose pose{};
};

inline Pose predict(const Pose & state, double v, double w, double age)
{
  Pose predicted = state;
  double dx=0.0, dy=0.0;
  yaw_math::integrateArc(state.yaw, v, w, age, dx, dy);
  predicted.x += dx;
  predicted.y += dy;
  predicted.yaw = yaw_math::wrapDifference(state.yaw + w*age);
  return predicted;
}

inline Step reconcile(const Pose & published, const Pose & reference,
  double dt, double age, double reference_skew,
  bool trusted, const Policy & policy)
{
  Step out;
  out.pose = published;
  if (!policy.enabled || !trusted || !std::isfinite(dt) || dt <= 0.0 || dt > 0.1 ||
      !std::isfinite(age) || age < 0.0 || age > policy.max_age_sec ||
      !std::isfinite(reference_skew) ||
      std::abs(reference_skew) > policy.max_reference_skew_sec ||
      !std::isfinite(policy.gain) || policy.gain <= 0.0 ||
      !std::isfinite(policy.max_linear_rate) || policy.max_linear_rate <= 0.0 ||
      !std::isfinite(policy.max_angular_rate) || policy.max_angular_rate <= 0.0 ||
      !std::isfinite(policy.max_error_m) || policy.max_error_m <= 0.0 ||
      !std::isfinite(policy.max_error_rad) || policy.max_error_rad <= 0.0 ||
      !std::isfinite(published.x) || !std::isfinite(published.y) ||
      !std::isfinite(published.yaw) || !std::isfinite(reference.x) ||
      !std::isfinite(reference.y) || !std::isfinite(reference.yaw)) {return out;}

  // Right-invariant group error Log(T_pub^{-1} T_ref).
  const double angle = yaw_math::wrapDifference(reference.yaw - published.yaw);
  const double dx = reference.x - published.x, dy = reference.y - published.y;
  const double c = std::cos(published.yaw), s = std::sin(published.yaw);
  const double body_x = c*dx + s*dy, body_y = -s*dx + c*dy;
  const double half = angle * 0.5;
  const double sinch = yaw_math::sinc(half);
  if (!std::isfinite(sinch) || std::abs(sinch) < 1e-7) {return out;}
  const double ch = std::cos(half), sh = std::sin(half);
  const double log_x = (ch*body_x + sh*body_y)/sinch;
  const double log_y = (-sh*body_x + ch*body_y)/sinch;
  const double norm = std::hypot(log_x, log_y);
  if (!std::isfinite(norm) || !std::isfinite(angle) ||
      norm > policy.max_error_m || std::abs(angle) > policy.max_error_rad) {return out;}
  out.distance_error = norm;
  out.yaw_error = angle;
  const double gain = std::clamp(policy.gain, 0.0, 1.0);
  double step_x = gain * log_x, step_y = gain * log_y;
  const double step_norm = std::hypot(step_x, step_y);
  const double cap = policy.max_linear_rate*dt;
  if (step_norm > cap) {
    const double factor = cap/step_norm;
    step_x *= factor;
    step_y *= factor;
  }
  const double step_angle = std::clamp(gain*angle,
      -policy.max_angular_rate*dt, policy.max_angular_rate*dt);
  // Exp_SE(2) applied in the current body frame.
  const double shalf = 0.5*step_angle;
  const double scale = yaw_math::sinc(shalf);
  const double exp_x = scale*(std::cos(shalf)*step_x-std::sin(shalf)*step_y);
  const double exp_y = scale*(std::sin(shalf)*step_x+std::cos(shalf)*step_y);
  out.pose.x += c*exp_x-s*exp_y;
  out.pose.y += s*exp_x+c*exp_y;
  out.pose.yaw = yaw_math::wrapDifference(published.yaw+step_angle);
  out.applied_distance = std::hypot(step_x, step_y);
  out.applied_yaw = step_angle;
  out.applied = out.applied_distance > 1e-12 || std::abs(step_angle) > 1e-12;
  return out;
}
}  // namespace mhe_fusion::se2_correct
