#include "mhe_sensor_fusion/covariance_calibration.hpp"
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "mhe_sensor_fusion/stability_guards.hpp"
#include "mhe_sensor_fusion/solver_fallback_policy.hpp"
#include "mhe_sensor_fusion/rt_budget.hpp"
#include "mhe_sensor_fusion/fast_covariance.hpp"
#include "mhe_sensor_fusion/low_latency_output.hpp"
#include "mhe_sensor_fusion/time_aligned_output.hpp"
#include "mhe_sensor_fusion/analytic_factors.hpp"
#include "mhe_sensor_fusion/se2_yaw_math.hpp"
#include "mhe_sensor_fusion/se2_error_correction.hpp"
#include "mhe_sensor_fusion/gyro_increment.hpp"
#include "mhe_sensor_fusion/block_schur.hpp"
#include "mhe_sensor_fusion/rank_aware_prior.hpp"
#include "mhe_sensor_fusion/marginal_factor_selector.hpp"

#include <ceres/ceres.h>
#include <ceres/covariance.h>
#include <ceres/crs_matrix.h>
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace mhe_fusion
{

constexpr int NX = 9;

enum StateIndex
{
  PX = 0,
  PY,
  YAW,
  V,
  W,
  BG,
  BA,
  SL,
  SR
};

using VecX = Eigen::Matrix<double, NX, 1>;
using MatX = Eigen::Matrix<double, NX, NX>;

struct MarginalPrior
{
  bool valid{false};
  VecX x_ref{VecX::Zero()};
  MatX sqrt_info{MatX::Identity()};
  VecX offset{VecX::Zero()};
};

struct ArrivalCost
{
  ArrivalCost(const std::array<double, NX> & prior,
              const std::array<double, NX> & sigma, bool so2_yaw = false)
  : prior_(prior), sigma_(sigma), so2_yaw_(so2_yaw) {}

  template<typename T>
  bool operator()(const T * const x, T * residual) const
  {
    for (int i = 0; i < NX; ++i) {
      const T diff = x[i] - T(prior_[i]);
      residual[i] = (so2_yaw_ && i == YAW ?
        yaw_math::wrapDifference(diff) : diff) / T(sigma_[i]);
    }
    return true;
  }

  std::array<double, NX> prior_;
  std::array<double, NX> sigma_;
  bool so2_yaw_{false};
};

struct ProcessCost
{
  ProcessCost(double dt, const std::array<double, NX> & sigma,
    bool exact_arc = false, bool so2_yaw = false)
  : dt_(dt), sigma_(sigma), exact_arc_(exact_arc), so2_yaw_(so2_yaw) {}

  template<typename T>
  bool operator()(const T * const x0, const T * const x1, T * residual) const
  {
    const T dt = T(dt_);
    const T v_bar = T(0.5) * (x0[V] + x1[V]);
    const T w_bar = T(0.5) * (x0[W] + x1[W]);
    const T yaw_mid = x0[YAW] + T(0.5) * dt * w_bar;

    T dx, dy;
    if (exact_arc_) {
      yaw_math::integrateArc(x0[YAW], v_bar, w_bar, dt, dx, dy);
    } else {
      dx = dt * v_bar * ceres::cos(yaw_mid);
      dy = dt * v_bar * ceres::sin(yaw_mid);
    }
    const T yaw_pred = x0[YAW] + dt * w_bar;

    residual[PX] = (x1[PX] - x0[PX] - dx) / T(sigma_[PX]);
    residual[PY] = (x1[PY] - x0[PY] - dy) / T(sigma_[PY]);
    const T yaw_error = x1[YAW] - yaw_pred;
    residual[YAW] = (so2_yaw_ ? yaw_math::wrapDifference(yaw_error) :
      yaw_error) / T(sigma_[YAW]);
    residual[V] = (x1[V] - x0[V]) / T(sigma_[V]);
    residual[W] = (x1[W] - x0[W]) / T(sigma_[W]);
    residual[BG] = (x1[BG] - x0[BG]) / T(sigma_[BG]);
    residual[BA] = (x1[BA] - x0[BA]) / T(sigma_[BA]);
    residual[SL] = (x1[SL] - x0[SL]) / T(sigma_[SL]);
    residual[SR] = (x1[SR] - x0[SR]) / T(sigma_[SR]);
    return true;
  }

  double dt_;
  std::array<double, NX> sigma_;
  bool exact_arc_{false};
  bool so2_yaw_{false};
};

struct WheelPairCost
{
  WheelPairCost(double wheel_left, double wheel_right, double wheel_separation,
                double sigma_left, double sigma_right)
  : wheel_left_(wheel_left), wheel_right_(wheel_right),
    wheel_separation_(wheel_separation), sigma_left_(sigma_left), sigma_right_(sigma_right) {}

  template<typename T>
  bool operator()(const T * const x, T * residual) const
  {
    const T half_track = T(0.5 * wheel_separation_);
    const T v_left_true = x[V] - half_track * x[W];
    const T v_right_true = x[V] + half_track * x[W];

    const T v_left_encoder_pred = (T(1.0) + x[SL]) * v_left_true;
    const T v_right_encoder_pred = (T(1.0) + x[SR]) * v_right_true;

    residual[0] = (v_left_encoder_pred - T(wheel_left_)) / T(sigma_left_);
    residual[1] = (v_right_encoder_pred - T(wheel_right_)) / T(sigma_right_);
    return true;
  }

  double wheel_left_;
  double wheel_right_;
  double wheel_separation_;
  double sigma_left_;
  double sigma_right_;
};

struct GyroCost
{
  GyroCost(double gyro, double sigma) : gyro_(gyro), sigma_(sigma) {}

  template<typename T>
  bool operator()(const T * const x, T * residual) const
  {
    residual[0] = (x[W] + x[BG] - T(gyro_)) / T(sigma_);
    return true;
  }

  double gyro_;
  double sigma_;
};

struct AccelCost
{
  AccelCost(double accel, double dt, double sigma)
  : accel_(accel), dt_(dt), sigma_(sigma) {}

  template<typename T>
  bool operator()(const T * const x0, const T * const x1, T * residual) const
  {
    const T acceleration = (x1[V] - x0[V]) / T(dt_);
    residual[0] = (acceleration + x1[BA] - T(accel_)) / T(sigma_);
    return true;
  }

  double accel_;
  double dt_;
  double sigma_;
};

struct ZeroMotionCost
{
  ZeroMotionCost(double sigma_v, double sigma_w)
  : sigma_v_(sigma_v), sigma_w_(sigma_w) {}

  template<typename T>
  bool operator()(const T * const x, T * residual) const
  {
    residual[0] = x[V] / T(sigma_v_);
    residual[1] = x[W] / T(sigma_w_);
    return true;
  }

  double sigma_v_;
  double sigma_w_;
};

struct WheelSlipPriorCost
{
  WheelSlipPriorCost(
    double common_reference, double differential_reference,
    double sigma_common, double sigma_diff)
  : common_reference_(common_reference), differential_reference_(differential_reference),
    sigma_common_(sigma_common), sigma_diff_(sigma_diff) {}

  template<typename T>
  bool operator()(const T * const x, T * residual) const
  {
    const T common = T(0.5) * (x[SL] + x[SR]);
    const T differential = T(0.5) * (x[SR] - x[SL]);
    residual[0] = (common - T(common_reference_)) / T(sigma_common_);
    residual[1] = (differential - T(differential_reference_)) / T(sigma_diff_);
    return true;
  }

  double common_reference_;
  double differential_reference_;
  double sigma_common_;
  double sigma_diff_;
};

struct CommonSlipAccelCost
{
  CommonSlipAccelCost(double imu_accel, double wheel_accel, double sigma)
  : imu_accel_(imu_accel), wheel_accel_(wheel_accel), sigma_(sigma) {}

  template<typename T>
  bool operator()(const T * const x, T * residual) const
  {
    const T common = T(0.5) * (x[SL] + x[SR]);
    const T true_accel = T(imu_accel_) - x[BA];
    // For slowly varying common slip: d(v_enc)/dt ~= (1+s_c) d(v_true)/dt.
    const T wheel_accel_pred = (T(1.0) + common) * true_accel;
    residual[0] = (wheel_accel_pred - T(wheel_accel_)) / T(sigma_);
    return true;
  }

  double imu_accel_;
  double wheel_accel_;
  double sigma_;
};

// Evaluate hot factors with analytic Jacobians; retain AutoDiff for A/B.
class AnalyticProcessCost final : public ceres::SizedCostFunction<NX, NX, NX>
{
public:
  AnalyticProcessCost(double dt, const std::array<double, NX> & sigma,
    bool exact_arc = false, bool so2_yaw = false)
  : kernel_(dt, sigma, exact_arc, so2_yaw) {}
  bool Evaluate(double const * const * parameters, double * residuals,
                double ** jacobians) const override
  {
    return kernel_.evaluate(parameters[0], parameters[1], residuals,
      jacobians ? jacobians[0] : nullptr,
      jacobians ? jacobians[1] : nullptr);
  }
private:
  analytic::ProcessKernel kernel_;
};

class AnalyticWheelPairCost final : public ceres::SizedCostFunction<2, NX>
{
public:
  AnalyticWheelPairCost(double left, double right, double separation,
                        double sigma_left, double sigma_right)
  : kernel_(left, right, separation, sigma_left, sigma_right) {}
  bool Evaluate(double const * const * parameters, double * residuals,
                double ** jacobians) const override
  {
    return kernel_.evaluate(parameters[0], residuals,
      jacobians ? jacobians[0] : nullptr);
  }
private:
  analytic::WheelKernel kernel_;
};

// One angular-rate sample produces exactly one interval-increment factor.
// It replaces its unary GyroCost, preventing reuse of the same measurement
// likelihood. The rate is assumed constant over the preceding short interval.
// The modeled quadrature mismatch is included in sigma_model_rate.
class AnalyticGyroIncrementCost final : public ceres::SizedCostFunction<1, NX, NX>
{
public:
  AnalyticGyroIncrementCost(double gyro_rate, double dt,
    double sigma_gyro_rate, double sigma_model_rate)
  : kernel_(gyro_rate, dt, sigma_gyro_rate, sigma_model_rate) {}

  bool Evaluate(double const * const * parameters, double * residuals,
    double ** jacobians) const override
  {
    return kernel_.evaluate(parameters[0], parameters[1], residuals,
      jacobians ? jacobians[0] : nullptr,
      jacobians ? jacobians[1] : nullptr);
  }
private:
  mhe_sensor_fusion::gyro_increment::Kernel kernel_;
};

class MarginalPriorCost : public ceres::SizedCostFunction<NX, NX>
{
public:
  MarginalPriorCost(const VecX & x_ref, const MatX & sqrt_info,
    const VecX & offset, bool so2_yaw = false)
  : x_ref_(x_ref), sqrt_info_(sqrt_info), offset_(offset),
    so2_yaw_(so2_yaw) {}

  bool Evaluate(double const * const * parameters,
                double * residuals,
                double ** jacobians) const override
  {
    Eigen::Map<const VecX> x(parameters[0]);
    Eigen::Map<VecX> r(residuals);
    VecX dx = x - x_ref_;
    if (so2_yaw_) { dx[YAW] = yaw_math::wrapDifference(dx[YAW]); }
    r = sqrt_info_ * dx + offset_;

    if (jacobians && jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, NX, NX, Eigen::RowMajor>> J(jacobians[0]);
      J = sqrt_info_;
    }
    return true;
  }

private:
  VecX x_ref_;
  MatX sqrt_info_;
  VecX offset_;
  bool so2_yaw_{false};
};

class MheFusionNode : public rclcpp::Node
{
public:
  MheFusionNode()
  : Node("mhe_sensor_fusion")
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom/unfiltered");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data");
    output_topic_ = declare_parameter<std::string>("output_topic", "/odom");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);

    wheel_separation_ = declare_parameter<double>("robot.wheel_separation", 0.30);

    frequency_ = declare_parameter<double>("frequency", 100.0);
    window_size_ = declare_parameter<int>("window_size", 12);
    max_iterations_ = declare_parameter<int>("max_iterations", 4);
    solver_budget_ms_ = declare_parameter<double>("solver_budget_ms", 6.0);
    transform_time_offset_ = declare_parameter<double>("transform_time_offset", 0.02);

    // Batch all sensor events drained on one publisher tick and solve once.
    // This prevents 2-4 complete nonlinear solves in the same 10 ms cycle.
    batch_solve_enabled_ = declare_parameter<bool>("solver.batch_events", true);
    dense_qr_max_states_ = declare_parameter<int>("solver.dense_qr_max_states", 16);
    prefer_normal_cholesky_ = declare_parameter<bool>(
      "solver.prefer_normal_cholesky", true);
    analytic_factors_enabled_ = declare_parameter<bool>(
      "solver.analytic_factors_enabled", true);
    // SO(2) yaw residual avoids a false 2*pi discontinuity at angle wrapping.
    // Exact SE(2) arc removes the midpoint chord approximation when turning.
    // Both default off in the node for safe legacy comparison.
    so2_yaw_residual_enabled_ = declare_parameter<bool>(
      "solver.so2_yaw_residual_enabled", false);
    exact_se2_motion_enabled_ = declare_parameter<bool>(
      "solver.exact_se2_motion_enabled", false);
    // Experimental one-sample SO(2) gyro increment replacing (not duplicating)
    // its unary gyro factor. Keeps adjacent-state sparsity for Block-Schur.
    gyro_increment_factor_enabled_ = declare_parameter<bool>(
      "solver.gyro_increment_factor_enabled", false);
    gyro_increment_model_sigma_ = std::max(0.01, declare_parameter<double>(
      "solver.gyro_increment_model_sigma", 0.12));
    gyro_increment_max_dt_ = std::clamp(declare_parameter<double>(
      "solver.gyro_increment_max_dt", 0.030), 0.001, 0.10);
    // Opt-in at the node default level. Shipped Gazebo YAML explicitly enables
    // these optimizations; legacy custom YAML keeps its original graph path.
    incremental_graph_enabled_ = declare_parameter<bool>("solver.incremental_graph_enabled", false);
    block_schur_enabled_ = declare_parameter<bool>("solver.block_schur_enabled", false);
    rank_aware_prior_enabled_ = declare_parameter<bool>(
      "solver.rank_aware_prior_enabled", false);
    prior_eigen_relative_cutoff_ = declare_parameter<double>(
      "solver.prior_eigen_relative_cutoff", 1e-9);
    prior_max_discarded_gradient_fraction_ = declare_parameter<double>(
      "solver.prior_max_discarded_gradient_fraction", 1e-4);
    // QR fallback shares the FIRST solve's soft time budget. Endpoint
    // rejection is physical validation, not normally a numerical failure.
    qr_fallback_min_remaining_ms_ = declare_parameter<double>(
      "solver.qr_fallback_min_remaining_ms", 1.0);
    qr_fallback_on_endpoint_jump_ = declare_parameter<bool>(
      "solver.qr_fallback_on_endpoint_jump", false);
    rt_deadline_ms_ = declare_parameter<double>("rt.deadline_ms", 10.0);

    // Time-aligned asynchronous fusion. Sensor measurements are inserted at
    // their own header timestamps instead of being forced onto the 100 Hz
    // publisher grid. The publisher remains a smooth fixed-rate propagator.
    horizon_duration_sec_ = declare_parameter<double>("timing.horizon_duration", 0.12);
    max_horizon_states_ = declare_parameter<int>("timing.max_horizon_states", 32);
    max_events_per_tick_ = declare_parameter<int>("timing.max_events_per_tick", 16);
    max_event_queue_size_ = declare_parameter<int>("timing.max_event_queue_size", 256);
    merge_tolerance_sec_ = declare_parameter<double>("timing.merge_tolerance", 0.0008);
    out_of_order_tolerance_sec_ = declare_parameter<double>("timing.out_of_order_tolerance", 0.002);
    reset_gap_sec_ = declare_parameter<double>("timing.reset_gap", 0.25);
    sensor_timeout_sec_ = declare_parameter<double>("timing.sensor_timeout", 0.15);
    timestamp_aligned_output_enabled_ = declare_parameter<bool>(
      "timing.timestamp_aligned_output_enabled", true);
    max_propagation_gap_sec_ = declare_parameter<double>(
      "timing.max_propagation_gap", 0.05);
    future_tolerance_sec_ = declare_parameter<double>(
      "timing.future_tolerance", 0.002);
    require_odom_for_init_ = declare_parameter<bool>("timing.require_odom_for_init", true);

    output_linear_deadband_ = declare_parameter<double>("output.linear_deadband", 0.002);
    output_angular_deadband_ = declare_parameter<double>("output.angular_deadband", 0.004);

    // Output shaping is intentionally applied only to v/w used by the realtime
    // propagator.  The optimized trajectory itself is never low-pass filtered.
    // This removes high-frequency endpoint velocity chatter without reintroducing
    // x/y/yaw jumps or a slow pose filter. Limits are deliberately well above
    // the normal robot dynamics, so commanded acceleration passes with little lag.
    output_smoothing_enabled_ = declare_parameter<bool>("output.smoothing.enabled", true);
    // Independent channels: preserve linear velocity smoothing without delaying
    // heading during Gazebo rotate/start/stop transients. Master switch remains
    // backward compatible; angular bypass is the new safe default.
    output_smoothing_linear_enabled_ = declare_parameter<bool>(
      "output.smoothing.linear_enabled", true);
    output_smoothing_angular_enabled_ = declare_parameter<bool>(
      "output.smoothing.angular_enabled", false);
    output_linear_tau_ = declare_parameter<double>("output.smoothing.linear_tau", 0.030);
    output_angular_tau_ = declare_parameter<double>("output.smoothing.angular_tau", 0.018);

    // Stability guard for the published v/w only. A 3-sample median rejects
    // isolated one-cycle optimizer jumps. Adaptive tau then applies stronger
    // filtering to tiny corrections (typical solve-to-solve chatter) while
    // automatically using a much smaller time constant for real maneuvers.
    // This keeps odom smooth without low-pass filtering x/y/yaw directly.
    output_median3_enabled_ = declare_parameter<bool>(
      "output.smoothing.median3_enabled", true);
    output_adaptive_tau_enabled_ = declare_parameter<bool>(
      "output.smoothing.adaptive_tau_enabled", true);
    output_linear_quiet_tau_ = declare_parameter<double>(
      "output.smoothing.linear_quiet_tau", 0.060);
    output_linear_fast_tau_ = declare_parameter<double>(
      "output.smoothing.linear_fast_tau", 0.020);
    output_angular_quiet_tau_ = declare_parameter<double>(
      "output.smoothing.angular_quiet_tau", 0.040);
    output_angular_fast_tau_ = declare_parameter<double>(
      "output.smoothing.angular_fast_tau", 0.008);
    output_linear_transition_ = declare_parameter<double>(
      "output.smoothing.linear_transition", 0.060);
    output_angular_transition_ = declare_parameter<double>(
      "output.smoothing.angular_transition", 0.150);

    output_max_linear_accel_ = declare_parameter<double>("output.smoothing.max_linear_accel", 3.0);
    output_max_linear_decel_ = declare_parameter<double>("output.smoothing.max_linear_decel", 4.0);
    output_max_angular_accel_ = declare_parameter<double>("output.smoothing.max_angular_accel", 12.0);
    output_max_angular_decel_ = declare_parameter<double>("output.smoothing.max_angular_decel", 16.0);
    output_max_linear_jerk_ = declare_parameter<double>("output.smoothing.max_linear_jerk", 60.0);
    output_max_angular_jerk_ = declare_parameter<double>("output.smoothing.max_angular_jerk", 240.0);

    // Experimental, opt-in reference anchoring. Never abruptly snap the
    // published yaw to the Ceres endpoint, which has a different timestamp.
    // Disabled until verified against Gazebo ground truth/TF logs.
    yaw_feedback_policy_.enabled = declare_parameter<bool>(
      "output.yaw_feedback.enabled", false);
    yaw_feedback_policy_.gain = declare_parameter<double>(
      "output.yaw_feedback.gain", 0.15);
    yaw_feedback_policy_.max_rate_rad_s = declare_parameter<double>(
      "output.yaw_feedback.max_rate", 0.20);
    yaw_feedback_policy_.max_error_rad = declare_parameter<double>(
      "output.yaw_feedback.max_error", 0.20);
    yaw_feedback_policy_.max_sensor_age_sec = declare_parameter<double>(
      "output.yaw_feedback.max_sensor_age", 0.04);
    yaw_feedback_policy_.max_reference_skew_sec = declare_parameter<double>(
      "output.yaw_feedback.max_reference_skew", 0.002);
    yaw_feedback_policy_.min_confidence = declare_parameter<double>(
      "output.yaw_feedback.min_confidence", 0.35);
    yaw_feedback_policy_.max_gyro_prefit_nis = declare_parameter<double>(
      "output.yaw_feedback.max_gyro_prefit_nis", 16.0);
    yaw_feedback_policy_.require_covariance = declare_parameter<bool>(
      "output.yaw_feedback.require_covariance", false);
    yaw_feedback_policy_.max_yaw_variance = declare_parameter<double>(
      "output.yaw_feedback.max_yaw_variance", 0.10);
    yaw_feedback_policy_.max_covariance_age_solves = static_cast<unsigned int>(std::max<int64_t>(0,
      declare_parameter<int>("output.yaw_feedback.max_covariance_age_solves", 80)));

    // SE(2) right-invariant correction is opt-in, mutually exclusive with the
    // legacy yaw-only feedback; never allow two pose correction loops.
    se2_correction_policy_.enabled = declare_parameter<bool>(
      "output.se2_correction.enabled", false);
    se2_correction_policy_.gain = declare_parameter<double>(
      "output.se2_correction.gain", 0.12);
    se2_correction_policy_.max_linear_rate = declare_parameter<double>(
      "output.se2_correction.max_linear_rate", 0.03);
    se2_correction_policy_.max_angular_rate = declare_parameter<double>(
      "output.se2_correction.max_angular_rate", 0.12);
    se2_correction_policy_.max_error_m = declare_parameter<double>(
      "output.se2_correction.max_error_m", 0.15);
    se2_correction_policy_.max_error_rad = declare_parameter<double>(
      "output.se2_correction.max_error_rad", 0.20);
    se2_correction_policy_.max_age_sec = declare_parameter<double>(
      "output.se2_correction.max_age_sec", 0.04);
    se2_correction_policy_.max_reference_skew_sec = declare_parameter<double>(
      "output.se2_correction.max_reference_skew_sec", 0.002);
    if (se2_correction_policy_.enabled && yaw_feedback_policy_.enabled) {
      RCLCPP_WARN(get_logger(),
        "Both SE2 correction and yaw feedback requested; disabling SE2 correction");
      se2_correction_policy_.enabled = false;
    }

    stationary_enabled_ = declare_parameter<bool>("stationary.enabled", true);
    stationary_v_threshold_ = declare_parameter<double>("stationary.linear_velocity_threshold", 0.012);
    stationary_w_threshold_ = declare_parameter<double>("stationary.angular_velocity_threshold", 0.025);
    stationary_gyro_threshold_ = declare_parameter<double>("stationary.gyro_threshold", 0.025);
    stationary_accel_threshold_ = declare_parameter<double>("stationary.accel_threshold", 0.18);
    stationary_min_samples_ = declare_parameter<int>("stationary.min_samples", 4);
    stationary_sigma_v_ = declare_parameter<double>("stationary.sigma_v", 0.004);
    stationary_sigma_w_ = declare_parameter<double>("stationary.sigma_w", 0.008);

    process_sigma_v_ = declare_parameter<double>("process.sigma_v", 0.10);
    process_sigma_w_ = declare_parameter<double>("process.sigma_w", 0.22);

    gyro_scale_ = declare_parameter<double>("gyro_z_scale", 1.0);
    accel_scale_ = declare_parameter<double>("accel_x_scale", 1.0);
    accel_lpf_tau_ = declare_parameter<double>("accel_lpf_tau", 0.05);

    accel_gravity_compensation_ = declare_parameter<bool>(
      "accel.gravity_compensation", false);
    gravity_mps2_ = declare_parameter<double>("accel.gravity_mps2", 9.80665);
    accel_axis_ = declare_parameter<std::vector<double>>(
      "accel.axis", std::vector<double>{1.0, 0.0, 0.0});
    if (accel_axis_.size() != 3) {
      RCLCPP_WARN(get_logger(), "accel.axis must contain 3 values; using [1,0,0]");
      accel_axis_ = {1.0, 0.0, 0.0};
    }
    const double accel_axis_norm = std::sqrt(
      accel_axis_[0] * accel_axis_[0] + accel_axis_[1] * accel_axis_[1] +
      accel_axis_[2] * accel_axis_[2]);
    if (accel_axis_norm < 1e-9) {
      accel_axis_ = {1.0, 0.0, 0.0};
    } else {
      for (auto & value : accel_axis_) {
        value /= accel_axis_norm;
      }
    }
    accel_auto_zero_enabled_ = declare_parameter<bool>("accel.auto_zero.enabled", true);
    accel_auto_zero_alpha_ = declare_parameter<double>("accel.auto_zero.alpha", 0.02);
    accel_auto_zero_max_abs_offset_ = declare_parameter<double>(
      "accel.auto_zero.max_abs_offset", 12.0);
    accel_max_jerk_ = declare_parameter<double>("accel.max_jerk", 30.0);
    accel_loss_delta_ = declare_parameter<double>("accel.huber_delta", 1.5);

    sigma_wheel_left_base_ = declare_parameter<double>("sigma_wheel_left", 0.025);
    sigma_wheel_right_base_ = declare_parameter<double>("sigma_wheel_right", 0.025);
    sigma_gyro_base_ = declare_parameter<double>("sigma_gyro", 0.025);
    sigma_accel_base_ = declare_parameter<double>("sigma_accel", 0.35);

    adaptive_enabled_ = declare_parameter<bool>("adaptive.enabled", true);
    adaptive_alpha_ = declare_parameter<double>("adaptive.ewma_alpha", 0.05);
    max_wheel_r_scale_ = declare_parameter<double>("adaptive.max_wheel_r_scale", 3.0);
    max_sensor_r_scale_ = declare_parameter<double>("adaptive.max_sensor_r_scale", 3.0);
    max_q_scale_ = declare_parameter<double>("adaptive.max_q_scale", 5.0);
    qv_gain_ = declare_parameter<double>("adaptive.qv_gain", 1.5);
    qw_gain_ = declare_parameter<double>("adaptive.qw_gain", 1.5);
    accel_reference_ = declare_parameter<double>("adaptive.accel_reference", 0.50);
    angular_accel_reference_ = declare_parameter<double>("adaptive.angular_accel_reference", 1.50);
    adaptive_q_scale_alpha_ = declare_parameter<double>("adaptive.q_scale_alpha", 0.08);

    // Hysteresis prevents R from toggling every frame near the NIS threshold.
    adaptive_nis_high_ = declare_parameter<double>("adaptive.nis_high", 4.0);
    adaptive_nis_low_ = declare_parameter<double>("adaptive.nis_low", 2.0);
    adaptive_enter_samples_ = declare_parameter<int>("adaptive.enter_samples", 5);
    adaptive_exit_samples_ = declare_parameter<int>("adaptive.exit_samples", 20);
    adaptive_scale_step_up_ = declare_parameter<double>("adaptive.scale_step_up", 0.20);
    adaptive_scale_step_down_ = declare_parameter<double>("adaptive.scale_step_down", 0.04);

    // Approximate prediction uncertainty used by the pre-fit normalized
    // innovation statistic.  MHE does not maintain a recursive EKF covariance,
    // so S = H P^- H^T + R is approximated by R plus a conservative model
    // prediction variance.  This keeps health gating sensitive to real faults
    // without treating normal robot acceleration as a sensor failure.
    prefit_wheel_model_sigma_ = declare_parameter<double>(
      "adaptive.prefit_wheel_model_sigma", 0.040);
    prefit_gyro_model_sigma_ = declare_parameter<double>(
      "adaptive.prefit_gyro_model_sigma", 0.025);
    prefit_accel_model_sigma_ = declare_parameter<double>(
      "adaptive.prefit_accel_model_sigma", 0.50);

    innovation_gating_enabled_ = declare_parameter<bool>("innovation_gate.enabled", true);
    gate_wheel_soft_nis_ = declare_parameter<double>("innovation_gate.wheel_soft_nis", 9.0);
    gate_wheel_hard_nis_ = declare_parameter<double>("innovation_gate.wheel_hard_nis", 64.0);
    gate_gyro_soft_nis_ = declare_parameter<double>("innovation_gate.gyro_soft_nis", 9.0);
    gate_gyro_hard_nis_ = declare_parameter<double>("innovation_gate.gyro_hard_nis", 64.0);
    gate_accel_soft_nis_ = declare_parameter<double>("innovation_gate.accel_soft_nis", 16.0);
    gate_accel_hard_nis_ = declare_parameter<double>("innovation_gate.accel_hard_nis", 100.0);
    gate_max_sigma_scale_ = declare_parameter<double>("innovation_gate.max_sigma_scale", 3.0);
    gate_warmup_accepted_solves_ = declare_parameter<int>(
      "innovation_gate.warmup_accepted_solves", 5);
    gate_large_gap_sec_ = declare_parameter<double>(
      "innovation_gate.large_gap_sec", 0.035);

    solver_cost_tolerance_ = declare_parameter<double>("solver.cost_tolerance", 1e-4);
    solver_max_endpoint_linear_accel_ = declare_parameter<double>(
      "solver.max_endpoint_linear_accel", 8.0);
    solver_max_endpoint_angular_accel_ = declare_parameter<double>(
      "solver.max_endpoint_angular_accel", 30.0);
    solver_linear_jump_margin_ = declare_parameter<double>(
      "solver.linear_jump_margin", 0.35);
    solver_angular_jump_margin_ = declare_parameter<double>(
      "solver.angular_jump_margin", 1.0);
    common_memory_max_prefit_nis_ = declare_parameter<double>(
      "slip.memory_max_prefit_nis", 9.0);

    slip_left_min_ = declare_parameter<double>("slip.left_min", -0.40);
    slip_left_max_ = declare_parameter<double>("slip.left_max", 1.50);
    slip_right_min_ = declare_parameter<double>("slip.right_min", -0.40);
    slip_right_max_ = declare_parameter<double>("slip.right_max", 1.50);
    slip_accel_observable_ = declare_parameter<double>("slip.accel_observable", 0.40);
    slip_angular_observable_ = declare_parameter<double>("slip.angular_observable", 0.35);
    sigma_slip_common_unobservable_ = declare_parameter<double>("slip.sigma_common_unobservable", 0.035);
    sigma_slip_common_active_ = declare_parameter<double>("slip.sigma_common_active", 0.15);
    sigma_slip_diff_unobservable_ = declare_parameter<double>("slip.sigma_diff_unobservable", 0.040);
    sigma_slip_diff_active_ = declare_parameter<double>("slip.sigma_diff_active", 0.30);
    sigma_sl_process_ = declare_parameter<double>("slip.sigma_left_process", 0.025);
    sigma_sr_process_ = declare_parameter<double>("slip.sigma_right_process", 0.025);
    slip_warm_start_decay_ = declare_parameter<double>("slip.warm_start_decay", 0.998);
    sigma_slip_common_hold_ = declare_parameter<double>("slip.sigma_common_hold", 0.080);
    common_slip_memory_tau_ = declare_parameter<double>("slip.common_memory_tau", 8.0);
    common_slip_learn_observability_ = declare_parameter<double>(
      "slip.common_learn_observability", 0.35);
    slip_min_speed_observable_ = declare_parameter<double>(
      "slip.min_speed_observable", 0.05);
    sigma_common_slip_accel_ = declare_parameter<double>(
      "slip.sigma_common_accel", 0.35);
    common_slip_accel_pair_max_age_ = declare_parameter<double>(
      "slip.accel_pair_max_age", 0.030);
    common_slip_memory_alpha_min_ = declare_parameter<double>(
      "slip.common_memory_alpha_min", 0.05);
    common_slip_memory_alpha_max_ = declare_parameter<double>(
      "slip.common_memory_alpha_max", 0.20);

    covariance_enabled_ = declare_parameter<bool>("covariance.enabled", true);
    covariance_update_every_n_ = declare_parameter<int>("covariance.update_every_n", 20);
    covariance_max_age_solves_ = declare_parameter<int>("covariance.max_age_solves", 60);
    covariance_rt_budget_ms_ = declare_parameter<double>("covariance.rt_budget_ms", 8.0);
    covariance_rt_reserve_ms_ = declare_parameter<double>("covariance.rt_reserve_ms", 2.5);
    covariance_defer_if_slow_ = declare_parameter<bool>("covariance.defer_if_slow", true);
    covariance_max_states_ = declare_parameter<int>("covariance.max_states", 20);
    covariance_min_variance_ = declare_parameter<double>("covariance.min_variance", 1e-9);
    covariance_max_variance_ = declare_parameter<double>("covariance.max_variance", 100.0);
    covariance_async_enabled_ = declare_parameter<bool>("covariance.async_enabled", true);
    covariance_snapshot_budget_ms_ = declare_parameter<double>("covariance.snapshot_budget_ms", 4.5);
    covariance_max_stale_solves_ = declare_parameter<int>("covariance.max_stale_solves", 120);
    covariance_jacobi_scaled_enabled_ = declare_parameter<bool>(
      "covariance.jacobi_scaled_enabled", false);
    covariance_svd_fallback_enabled_ = declare_parameter<bool>(
      "covariance.svd_fallback_enabled", true);
    covariance_minimum_rcond_ = declare_parameter<double>(
      "covariance.minimum_rcond", 1e-10);
    covariance_svd_relative_cutoff_ = declare_parameter<double>(
      "covariance.svd_relative_cutoff", 1e-11);
    covariance_maximum_svd_condition_ = declare_parameter<double>(
      "covariance.maximum_svd_condition", 1e10);
    pose_covariance_calibration_enabled_ = declare_parameter<bool>(
      "covariance.calibration.enabled", false);
    const auto pose_scales = declare_parameter<std::vector<double>>(
      "covariance.calibration.pose_std_scales", std::vector<double>{1.0, 1.0, 1.0});
    if (pose_scales.size() != 3) {
      throw std::invalid_argument("covariance.calibration.pose_std_scales must have 3 elements");
    }
    Eigen::Vector3d checked_scales(pose_scales[0], pose_scales[1], pose_scales[2]);
    // Validate even when disabled, so bad config is never silently accepted.
    (void)mhe_sensor_fusion::calibratePublishedPoseCovariance(
      Eigen::Matrix3d::Identity(), checked_scales);
    pose_covariance_std_scales_ = checked_scales;

    max_abs_v_ = declare_parameter<double>("limits.max_abs_v", 1.5);
    max_abs_w_ = declare_parameter<double>("limits.max_abs_w", 6.0);
    max_abs_gyro_ = declare_parameter<double>("limits.max_abs_gyro", 8.0);
    max_abs_accel_ = declare_parameter<double>("limits.max_abs_accel", 15.0);

    process_sigma_base_ = {
      0.010, 0.010, 0.008,
      process_sigma_v_, process_sigma_w_,
      0.002, 0.030,
      sigma_sl_process_, sigma_sr_process_
    };

    arrival_sigma_ = {
      0.030, 0.030, 0.025,
      0.060, 0.080,
      0.020, 0.250,
      0.080, 0.080
    };

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MheFusionNode::odomCallback, this, std::placeholders::_1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MheFusionNode::imuCallback, this, std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, 10);
    confidence_pub_ = create_publisher<std_msgs::msg::Float64>("/mhe/confidence", 10);
    innovation_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/mhe/innovation", 10);
    bias_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/mhe/bias", 10);
    solve_time_pub_ = create_publisher<std_msgs::msg::Float64>("/mhe/solve_time_ms", 10);
    marginal_condition_pub_ = create_publisher<std_msgs::msg::Float64>("/mhe/marginal_condition", 10);
    wheel_slip_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/mhe/wheel_slip", 10);
    sensor_health_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/mhe/sensor_health", 10);
    raw_nis_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/mhe/nis_raw", 10);
    postfit_residual_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/mhe/postfit_residual", 10);
    timing_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/mhe/timing", 10);
    time_alignment_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/mhe/time_alignment", 10);
    covariance_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/mhe/covariance_diag", 10);
    accel_status_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/mhe/accel_status", 10);
    output_status_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/mhe/output_status", 10);
    gating_status_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/mhe/gating_status", 10);
    solver_health_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/mhe/solver_health", 10);
    realtime_profile_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/mhe/rt_profile", 10);
    graph_status_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/mhe/graph_status", 10);
    covariance_worker_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/mhe/cov_worker_status", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    const auto period = std::chrono::duration<double>(1.0 / frequency_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MheFusionNode::timerCallback, this));

    RCLCPP_INFO(get_logger(),
      "MHE started: %.1f Hz output, %.3f s async horizon, wheel_separation=%.4f m",
      frequency_, horizon_duration_sec_, wheel_separation_);
  }

private:
  struct Measurement
  {
    // dt is retained for diagnostics/backward compatibility.  Optimizer
    // transition dt is always recomputed from adjacent state timestamps so a
    // merged asynchronous event cannot leave a stale transition interval.
    double dt{0.01};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    bool has_odom{false};
    bool has_gyro{false};
    bool has_accel{false};
    bool stationary{false};
    double odom_v{0.0};
    double odom_w{0.0};
    double wheel_left{0.0};
    double wheel_right{0.0};
    double gyro_z{0.0};
    double accel_x{0.0};

    // Snapshot adaptive weights when this measurement/transition arrives.
    // Historical factors keep their own weights instead of being reweighted
    // every time the newest sensor-health state changes.
    double r_scale_left{1.0};
    double r_scale_right{1.0};
    double r_scale_gyro{1.0};
    double r_scale_accel{1.0};
    double qv_scale{1.0};
    double qw_scale{1.0};

    // Slip observability and prior are snapshotted per state.  Historical
    // factors therefore keep the information available when they arrived.
    double common_slip_observability{0.0};
    double differential_slip_observability{0.0};
    double sigma_common_slip{0.035};
    double sigma_differential_slip{0.040};
    double common_slip_reference{0.0};
    double differential_slip_reference{0.0};
    bool has_common_slip_accel_pair{false};
    double paired_imu_accel{0.0};
    double paired_wheel_accel{0.0};
  };

  enum class SensorEventType
  {
    ODOM,
    IMU
  };

  struct SensorEvent
  {
    SensorEventType type{SensorEventType::ODOM};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    uint64_t sequence{0};
    nav_msgs::msg::Odometry odom;
    sensor_msgs::msg::Imu imu;
  };

  struct AdaptiveGate
  {
    bool degraded{false};
    int high_count{0};
    int low_count{0};
    double scale{1.0};
  };

  rclcpp::Time sanitizeStamp(const builtin_interfaces::msg::Time & stamp)
  {
    rclcpp::Time t(stamp, RCL_ROS_TIME);
    if (t.nanoseconds() <= 0) {
      t = now();
    }
    return t;
  }

  double extractLongitudinalAcceleration(
    const sensor_msgs::msg::Imu & imu, bool & gravity_compensated) const
  {
    tf2::Vector3 accel(
      accel_scale_ * imu.linear_acceleration.x,
      accel_scale_ * imu.linear_acceleration.y,
      accel_scale_ * imu.linear_acceleration.z);

    gravity_compensated = false;
    if (accel_gravity_compensation_ && imu.orientation_covariance[0] >= 0.0) {
      const auto & oq = imu.orientation;
      const double qnorm2 =
        oq.x * oq.x + oq.y * oq.y + oq.z * oq.z + oq.w * oq.w;
      if (std::isfinite(qnorm2) && qnorm2 > 1e-10) {
        tf2::Quaternion q(oq.x, oq.y, oq.z, oq.w);
        q.normalize();

        // ROS IMUs commonly report specific force.  At rest the measurement
        // is -g expressed in the sensor frame, so linear acceleration is
        // recovered by adding gravity expressed in that frame.
        const tf2::Vector3 g_world(0.0, 0.0, -gravity_mps2_);
        const tf2::Vector3 g_imu = tf2::quatRotate(q.inverse(), g_world);
        accel += g_imu;
        gravity_compensated = true;
      }
    }

    return accel.x() * accel_axis_[0] +
           accel.y() * accel_axis_[1] +
           accel.z() * accel_axis_[2];
  }

  void enqueueEvent(SensorEvent && event)
  {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    event.sequence = next_event_sequence_++;

    const size_t max_queue = static_cast<size_t>(std::max(1, max_event_queue_size_));
    if (sensor_event_queue_.size() >= max_queue) {
      sensor_event_queue_.pop_front();
      ++queue_drop_count_;
    }

    sensor_event_queue_.push_back(std::move(event));
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    SensorEvent event;
    event.type = SensorEventType::ODOM;
    event.stamp = sanitizeStamp(msg->header.stamp);
    event.odom = *msg;
    enqueueEvent(std::move(event));
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    SensorEvent event;
    event.type = SensorEventType::IMU;
    event.stamp = sanitizeStamp(msg->header.stamp);
    event.imu = *msg;
    enqueueEvent(std::move(event));
  }

  std::vector<SensorEvent> drainSensorEvents(const rclcpp::Time & ros_now)
  {
    std::vector<SensorEvent> events;

    {
      std::lock_guard<std::mutex> lock(sensor_mutex_);
      const size_t max_count = static_cast<size_t>(std::max(1, max_events_per_tick_));
      events.reserve(std::min(sensor_event_queue_.size(), max_count));
      size_t deferred_future = 0;
      auto it = sensor_event_queue_.begin();
      const auto tolerance_ns = static_cast<int64_t>(
        std::max(0.0, future_tolerance_sec_) * 1.0e9);
      // Do not process measurements from the future of the Gazebo /clock.
      // Scan the bounded deque instead of stopping at its first future event:
      // arrival order is not necessarily ROS timestamp order.
      while (it != sensor_event_queue_.end()) {
        const bool due = !timestamp_aligned_output_enabled_ ||
          mhe_sensor_fusion::time_align::eventIsDue(
            it->stamp.nanoseconds(), ros_now.nanoseconds(), tolerance_ns);
        if (!due) {
          ++deferred_future;
          ++it;
        } else if (events.size() < max_count) {
          events.push_back(std::move(*it));
          it = sensor_event_queue_.erase(it);
        } else {
          ++it;
        }
      }
      last_deferred_future_events_ = deferred_future;
      last_queue_depth_ = sensor_event_queue_.size();
    }

    std::stable_sort(
      events.begin(), events.end(),
      [](const SensorEvent & a, const SensorEvent & b) {
        if (a.stamp.nanoseconds() == b.stamp.nanoseconds()) {
          return a.sequence < b.sequence;
        }
        return a.stamp < b.stamp;
      });

    return events;
  }

  void updateNIS(
    double innovation, double innovation_sigma, double & raw_nis,
    double & ewma, bool & initialized)
  {
    // PRE-FIT normalized innovation statistic.  Keep both the instantaneous
    // value and its EWMA: raw NIS is useful for diagnostics while the smoothed
    // value drives hysteresis and adaptive R.
    const double n = innovation / std::max(innovation_sigma, 1e-8);
    raw_nis = std::min(n * n, 100.0);
    if (!initialized) {
      ewma = raw_nis;
      initialized = true;
    } else {
      ewma = (1.0 - adaptive_alpha_) * ewma + adaptive_alpha_ * raw_nis;
    }
  }

  double nisScale(double nis, double max_scale) const
  {
    return std::clamp(std::sqrt(std::max(1.0, nis)), 1.0, max_scale);
  }

  bool detectStationary(const Measurement & z)
  {
    if (!stationary_enabled_ || states_.empty()) {
      stationary_counter_ = 0;
      stationary_latched_ = false;
      return false;
    }

    const auto & x = states_.back();
    const bool gyro_quiet = !z.has_gyro ||
      std::abs(z.gyro_z - x[BG]) <= stationary_gyro_threshold_;
    const bool accel_quiet = !z.has_accel ||
      std::abs(z.accel_x - x[BA]) <= stationary_accel_threshold_;

    // At 100 Hz the odometry source may still be only 50 Hz. Do not drop the
    // stationary latch merely because this timer tick has no new odom sample.
    // A fresh IMU sample can still immediately break the latch if motion starts.
    if (!z.has_odom) {
      if (!gyro_quiet || !accel_quiet) {
        stationary_counter_ = 0;
        stationary_latched_ = false;
      }
      return stationary_latched_;
    }

    const bool wheel_quiet =
      std::abs(z.odom_v) <= stationary_v_threshold_ &&
      std::abs(z.odom_w) <= stationary_w_threshold_;

    if (wheel_quiet && gyro_quiet && accel_quiet) {
      stationary_counter_ = std::min(stationary_counter_ + 1, stationary_min_samples_);
    } else {
      stationary_counter_ = 0;
    }

    stationary_latched_ = stationary_counter_ >= stationary_min_samples_;
    return stationary_latched_;
  }

  void updateSlipObservability(Measurement & z)
  {
    if (states_.empty()) {
      return;
    }

    const auto & x = states_.back();
    const double speed = z.has_odom ? std::abs(z.odom_v) : std::abs(x[V]);
    const double speed_gate = std::clamp(
      speed / std::max(slip_min_speed_observable_, 1e-3), 0.0, 1.0);

    double accel_obs = 0.0;
    if (z.has_common_slip_accel_pair) {
      const double corrected_accel = std::abs(z.paired_imu_accel - x[BA]);
      accel_obs = std::clamp(
        corrected_accel / std::max(slip_accel_observable_, 1e-3), 0.0, 1.0);
    } else if (z.has_accel) {
      const double corrected_accel = std::abs(z.accel_x - x[BA]);
      accel_obs = std::clamp(
        corrected_accel / std::max(slip_accel_observable_, 1e-3), 0.0, 1.0);
    }

    const double wheel_accel_for_obs = z.has_common_slip_accel_pair ?
      z.paired_wheel_accel : wheel_accel_excitation_;
    const double wheel_accel_obs = std::clamp(
      std::abs(wheel_accel_for_obs) / std::max(slip_accel_observable_, 1e-3),
      0.0, 1.0);

    // Common-mode slip is only learnable when longitudinal motion is excited.
    // Combine IMU and wheel-speed excitation, then gate at very low speed.
    // A small IMU-only term keeps asynchronous IMU events useful until the
    // next odometry sample arrives.
    const double paired_excitation = std::sqrt(
      std::max(0.0, accel_obs * std::max(wheel_accel_obs, 0.25 * accel_obs)));
    common_slip_observability_ = std::clamp(speed_gate * paired_excitation, 0.0, 1.0);

    double rotation_level = 0.0;
    if (z.has_gyro) {
      rotation_level = std::abs(z.gyro_z - x[BG]);
    } else if (z.has_odom) {
      rotation_level = std::abs(z.odom_w);
    }

    differential_slip_observability_ = std::clamp(
      rotation_level / std::max(slip_angular_observable_, 1e-3), 0.0, 1.0);

    // Slowly forget a previously learned common slip instead of instantly
    // forcing it to zero when the robot returns to constant-speed motion.
    if (common_slip_memory_valid_) {
      double memory_dt = 0.0;
      if (common_slip_memory_last_stamp_.nanoseconds() > 0 &&
          z.stamp > common_slip_memory_last_stamp_) {
        memory_dt = (z.stamp - common_slip_memory_last_stamp_).seconds();
      }
      if (memory_dt > 0.0) {
        const double decay = std::exp(
          -memory_dt / std::max(common_slip_memory_tau_, 1e-3));
        common_slip_memory_ *= decay;
      }
    }
    if (z.stamp > common_slip_memory_last_stamp_) {
      common_slip_memory_last_stamp_ = z.stamp;
    }

    z.common_slip_observability = common_slip_observability_;
    z.differential_slip_observability = differential_slip_observability_;

    if (common_slip_observability_ >= common_slip_learn_observability_) {
      z.sigma_common_slip = sigma_slip_common_unobservable_ +
        common_slip_observability_ *
        (sigma_slip_common_active_ - sigma_slip_common_unobservable_);
      z.common_slip_reference = common_slip_memory_valid_ ? common_slip_memory_ : 0.0;
    } else if (common_slip_memory_valid_) {
      z.sigma_common_slip = sigma_slip_common_hold_;
      z.common_slip_reference = common_slip_memory_;
    } else {
      z.sigma_common_slip = sigma_slip_common_unobservable_;
      z.common_slip_reference = 0.0;
    }

    z.sigma_differential_slip = sigma_slip_diff_unobservable_ +
      differential_slip_observability_ *
      (sigma_slip_diff_active_ - sigma_slip_diff_unobservable_);
    z.differential_slip_reference = 0.0;
  }

  // Gating is evaluated once for each newly received measurement, BEFORE
  // warm start and BEFORE Ceres can fit the sample.  Gating uses the raw NIS,
  // while the slower adaptive sensor-health system uses EWMA NIS.
  bool applyInnovationGating(Measurement & z)
  {
    gate_state_last_.fill(0.0);
    if (!innovation_gating_enabled_ ||
        accepted_solution_count_ < static_cast<uint64_t>(
          std::max(0, gate_warmup_accepted_solves_))) {
      return z.has_odom || z.has_gyro || z.has_accel;
    }
    // A 40+ ms measurement gap can yield a large PRE-FIT residual during a
    // real maneuver even when the new sensor measurement is perfectly valid.
    // In that case use soft gating unless the discrepancy is also too large
    // for a plausible robot acceleration over the gap.
    const double interval = (z.stamp - last_measurement_stamp_).seconds();
    const bool large_gap = interval > gate_large_gap_sec_;
    auto decision = [&](double nis, double soft, double hard, size_t index,
                        double innovation, double max_plausible_gap_jump) {
      auto result = safety::classifyInnovation(
        nis, soft, hard, gate_max_sigma_scale_);
      if (large_gap && result.state == safety::GateState::Hard &&
          std::isfinite(innovation) &&
          std::abs(innovation) <= max_plausible_gap_jump) {
        result.state = safety::GateState::Soft;
        result.sigma_scale = gate_max_sigma_scale_;
      }
      gate_state_last_[index] = static_cast<double>(static_cast<int>(result.state));
      if (result.state == safety::GateState::Hard) {
        ++gate_reject_counts_[index];
      }
      return result;
    };
    if (z.has_odom) {
      // Current wheel factor is a JOINT two-component factor. If either side
      // is an outlier reject the pair; do not accidentally use the other side
      // with a misleading independent covariance assumption.
      const auto left = decision(raw_nis_wheel_left_, gate_wheel_soft_nis_,
                                 gate_wheel_hard_nis_, 0, prefit_e_left_, 0.6);
      const auto right = decision(raw_nis_wheel_right_, gate_wheel_soft_nis_,
                                  gate_wheel_hard_nis_, 1, prefit_e_right_, 0.6);
      if (left.state == safety::GateState::Hard ||
          right.state == safety::GateState::Hard) {
        z.has_odom = false;
        z.has_common_slip_accel_pair = false;
        // A rejected encoder packet must not be used as the start of a
        // wheel-acceleration derivative on the following accepted packet.
        have_last_odom_for_accel_ = false;
        wheel_accel_excitation_ = 0.0;
        latest_odom_quiet_ = false;
        stationary_counter_ = 0;
        stationary_latched_ = false;
      } else {
        z.r_scale_left *= left.sigma_scale;
        z.r_scale_right *= right.sigma_scale;
        if (left.state == safety::GateState::Soft ||
            right.state == safety::GateState::Soft) {
          z.has_common_slip_accel_pair = false;
        }
      }
    }
    if (z.has_gyro) {
      const auto gyro = decision(raw_nis_gyro_, gate_gyro_soft_nis_,
                                 gate_gyro_hard_nis_, 2, prefit_e_gyro_, 0.8);
      if (gyro.state == safety::GateState::Hard) {
        z.has_gyro = false;
      } else {
        z.r_scale_gyro *= gyro.sigma_scale;
      }
    }
    if (z.has_accel) {
      const auto accel = decision(raw_nis_accel_, gate_accel_soft_nis_,
                                  gate_accel_hard_nis_, 3, prefit_e_accel_, 4.0);
      if (accel.state == safety::GateState::Hard) {
        z.has_accel = false;
        z.has_common_slip_accel_pair = false;
        accel_filter_initialized_ = false;
        have_previous_corrected_accel_ = false;
        accel_zero_counter_ = 0;
      } else {
        z.r_scale_accel *= accel.sigma_scale;
        if (accel.state == safety::GateState::Soft) {
          z.has_common_slip_accel_pair = false;
        }
      }
    }
    return z.has_odom || z.has_gyro || z.has_accel;
  }

  void updateAdaptivePre(const Measurement & z)
  {
    if (!adaptive_enabled_ || states_.empty()) {
      return;
    }

    const auto & x = states_.back();

    if (z.has_accel) {
      const double corrected_accel = z.accel_x - x[BA];
      const double target = std::clamp(
        1.0 + qv_gain_ * std::abs(corrected_accel) / std::max(accel_reference_, 1e-3),
        1.0, max_q_scale_);
      const double alpha = std::clamp(adaptive_q_scale_alpha_, 0.01, 1.0);
      qv_scale_ = (1.0 - alpha) * qv_scale_ + alpha * target;
    }

    if (z.has_gyro) {
      const double gyro_corrected = z.gyro_z - x[BG];
      if (have_previous_gyro_) {
        double gyro_dt = z.dt;
        if (previous_gyro_stamp_.nanoseconds() > 0) {
          gyro_dt = (z.stamp - previous_gyro_stamp_).seconds();
        }
        gyro_dt = std::clamp(gyro_dt, 0.0005, 0.10);
        const double angular_accel = std::abs(gyro_corrected - previous_gyro_corrected_) /
                                     gyro_dt;
        const double target = std::clamp(
          1.0 + qw_gain_ * angular_accel / std::max(angular_accel_reference_, 1e-3),
          1.0, max_q_scale_);
          const double alpha = std::clamp(adaptive_q_scale_alpha_, 0.01, 1.0);
        qw_scale_ = (1.0 - alpha) * qw_scale_ + alpha * target;
      }
      previous_gyro_corrected_ = gyro_corrected;
      previous_gyro_stamp_ = z.stamp;
      have_previous_gyro_ = true;
    }
  }

  void updateAdaptiveGate(double nis, double max_scale, AdaptiveGate & gate)
  {
    if (!adaptive_enabled_) {
      gate = AdaptiveGate{};
      return;
    }

    if (!gate.degraded) {
      if (nis > adaptive_nis_high_) {
        gate.high_count++;
      } else {
        gate.high_count = 0;
      }
      gate.low_count = 0;

      if (gate.high_count >= std::max(1, adaptive_enter_samples_)) {
        gate.degraded = true;
        gate.high_count = 0;
      }
    } else {
      if (nis < adaptive_nis_low_) {
        gate.low_count++;
      } else {
        gate.low_count = 0;
      }
      gate.high_count = 0;

      if (gate.low_count >= std::max(1, adaptive_exit_samples_)) {
        gate.degraded = false;
        gate.low_count = 0;
      }
    }

    const double target = gate.degraded ? nisScale(nis, max_scale) : 1.0;
    const double delta = target - gate.scale;
    if (delta > 0.0) {
      gate.scale += std::min(delta, adaptive_scale_step_up_);
    } else {
      gate.scale += std::max(delta, -adaptive_scale_step_down_);
    }
    gate.scale = std::clamp(gate.scale, 1.0, max_scale);
  }

  void calculateConfidence()
  {
    const double mean_nis =
      (nis_wheel_left_ + nis_wheel_right_ + nis_gyro_ + nis_accel_) / 4.0;
    const double excess = std::max(0.0, mean_nis - 1.0);
    const double consistency = std::exp(-0.18 * excess);

    double slip_quality = 1.0;
    if (!states_.empty()) {
      const double diff_slip = std::abs(0.5 * (states_.back()[SR] - states_.back()[SL]));
      slip_quality = std::exp(-0.8 * diff_slip);
    }

    confidence_ = std::clamp(consistency * slip_quality, 0.0, 1.0);
    if (!last_solution_usable_) {
      confidence_ *= 0.25;
    }
  }

  void updatePrefitSensorHealth(const Measurement & z)
  {
    if (states_.empty()) {
      return;
    }

    // IMPORTANT: this function is called before warm-starting the new state and
    // before Ceres sees the incoming factor.  Therefore these innovations are
    // genuinely pre-fit rather than residuals that the optimizer has already
    // driven toward zero.
    const auto & x = states_.back();

    if (z.has_odom) {
      const double half_track = 0.5 * wheel_separation_;
      const double v_left_true = x[V] - half_track * x[W];
      const double v_right_true = x[V] + half_track * x[W];
      const double pred_left = (1.0 + x[SL]) * v_left_true;
      const double pred_right = (1.0 + x[SR]) * v_right_true;

      // Innovation convention: measurement - prior prediction.
      prefit_e_left_ = z.wheel_left - pred_left;
      prefit_e_right_ = z.wheel_right - pred_right;

      const double sigma_left = std::hypot(
        sigma_wheel_left_base_, prefit_wheel_model_sigma_);
      const double sigma_right = std::hypot(
        sigma_wheel_right_base_, prefit_wheel_model_sigma_);

      updateNIS(prefit_e_left_, sigma_left, raw_nis_wheel_left_, nis_wheel_left_, nis_left_initialized_);
      updateNIS(prefit_e_right_, sigma_right, raw_nis_wheel_right_, nis_wheel_right_, nis_right_initialized_);
      updateAdaptiveGate(nis_wheel_left_, max_wheel_r_scale_, gate_left_);
      updateAdaptiveGate(nis_wheel_right_, max_wheel_r_scale_, gate_right_);
      current_r_scale_left_ = gate_left_.scale;
      current_r_scale_right_ = gate_right_.scale;
    }

    if (z.has_gyro) {
      const double pred_gyro = x[W] + x[BG];
      prefit_e_gyro_ = z.gyro_z - pred_gyro;
      const double sigma = std::hypot(sigma_gyro_base_, prefit_gyro_model_sigma_);
      updateNIS(prefit_e_gyro_, sigma, raw_nis_gyro_, nis_gyro_, nis_gyro_initialized_);
      updateAdaptiveGate(nis_gyro_, max_sensor_r_scale_, gate_gyro_);
      current_r_scale_gyro_ = gate_gyro_.scale;
    }

    if (z.has_accel) {
      // Predict acceleration only from already-optimized history.  The current
      // accelerometer sample is never used to form its own prediction.
      double pred_accel = x[BA];
      if (states_.size() >= 2 && measurements_.size() >= 2) {
        const auto & x0 = states_[states_.size() - 2];
        double hist_dt =
          (measurements_.back().stamp - measurements_[measurements_.size() - 2].stamp).seconds();
        hist_dt = std::clamp(hist_dt, 0.0005, 0.10);
        pred_accel = (x[V] - x0[V]) / hist_dt + x[BA];
      }
      prefit_e_accel_ = z.accel_x - pred_accel;
      const double sigma = std::hypot(sigma_accel_base_, prefit_accel_model_sigma_);
      updateNIS(prefit_e_accel_, sigma, raw_nis_accel_, nis_accel_, nis_accel_initialized_);
      updateAdaptiveGate(nis_accel_, max_sensor_r_scale_, gate_accel_);
      current_r_scale_accel_ = gate_accel_.scale;
    }

    calculateConfidence();
  }

  void updatePostFitResiduals()
  {
    if (states_.empty() || measurements_.empty()) {
      return;
    }

    const auto & x = states_.back();
    const auto & z = measurements_.back();

    last_e_left_ = 0.0;
    last_e_right_ = 0.0;
    last_e_gyro_ = 0.0;
    last_e_accel_ = 0.0;

    if (z.has_odom) {
      const double half_track = 0.5 * wheel_separation_;
      const double v_left_true = x[V] - half_track * x[W];
      const double v_right_true = x[V] + half_track * x[W];
      const double wheel_left_pred = (1.0 + x[SL]) * v_left_true;
      const double wheel_right_pred = (1.0 + x[SR]) * v_right_true;
      last_e_left_ = wheel_left_pred - z.wheel_left;
      last_e_right_ = wheel_right_pred - z.wheel_right;
    }

    if (z.has_gyro) {
      last_e_gyro_ = x[W] + x[BG] - z.gyro_z;
    }

    if (z.has_accel && states_.size() >= 2 && measurements_.size() >= 2) {
      const auto & x0 = states_[states_.size() - 2];
      double dt =
        (measurements_.back().stamp - measurements_[measurements_.size() - 2].stamp).seconds();
      dt = std::clamp(dt, 0.0005, 0.10);
      const double acceleration = (x[V] - x0[V]) / dt;
      last_e_accel_ = acceleration + x[BA] - z.accel_x;
    }

    // Learn common-mode slip only during sufficiently observable motion and
    // at non-trivial ground speed.  The learned value becomes a slowly
    // decaying reference during subsequent constant-speed intervals, where
    // common slip is fundamentally weakly observable from wheel+IMU alone.
    if (last_solution_usable_ && z.has_common_slip_accel_pair &&
        z.common_slip_observability >= common_slip_learn_observability_ &&
        std::abs(x[V]) >= slip_min_speed_observable_ &&
        raw_nis_wheel_left_ <= common_memory_max_prefit_nis_ &&
        raw_nis_wheel_right_ <= common_memory_max_prefit_nis_ &&
        raw_nis_accel_ <= common_memory_max_prefit_nis_ &&
        !gate_left_.degraded && !gate_right_.degraded &&
        !gate_accel_.degraded) {
      const double common = 0.5 * (x[SL] + x[SR]);
      const double alpha = std::clamp(
        common_slip_memory_alpha_min_ +
        (common_slip_memory_alpha_max_ - common_slip_memory_alpha_min_) *
        z.common_slip_observability,
        std::min(common_slip_memory_alpha_min_, common_slip_memory_alpha_max_),
        std::max(common_slip_memory_alpha_min_, common_slip_memory_alpha_max_));
      if (!common_slip_memory_valid_) {
        common_slip_memory_ = common;
        common_slip_memory_valid_ = true;
      } else {
        common_slip_memory_ =
          (1.0 - alpha) * common_slip_memory_ + alpha * common;
      }
    }

    // Confidence uses pre-fit NIS, but update it again after solve so a failed
    // Ceres solution immediately receives the solver-validity penalty.
    calculateConfidence();
  }

  void resetHorizon(bool reset_published_pose)
  {
    invalidateGraph();
    states_.clear();
    measurements_.clear();
    marginal_prior_ = MarginalPrior{};
    last_measurement_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    accel_filter_initialized_ = false;
    accel_zero_counter_ = 0;
    have_previous_corrected_accel_ = false;
    have_last_odom_for_accel_ = false;
    wheel_accel_excitation_ = 0.0;
    latest_odom_quiet_ = false;
    have_last_imu_stamp_ = false;
    have_previous_gyro_ = false;
    previous_gyro_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    stationary_counter_ = 0;
    stationary_latched_ = false;

    // A discontinuity invalidates the innovation history.  Restart the health
    // EWMA and hysteresis rather than carrying stale degraded/healthy evidence
    // across a sensor gap or ROS-time jump.
    nis_wheel_left_ = 1.0;
    nis_wheel_right_ = 1.0;
    nis_gyro_ = 1.0;
    nis_accel_ = 1.0;
    raw_nis_wheel_left_ = 0.0;
    raw_nis_wheel_right_ = 0.0;
    raw_nis_gyro_ = 0.0;
    raw_nis_accel_ = 0.0;
    nis_left_initialized_ = false;
    nis_right_initialized_ = false;
    nis_gyro_initialized_ = false;
    nis_accel_initialized_ = false;
    gate_left_ = AdaptiveGate{};
    gate_right_ = AdaptiveGate{};
    gate_gyro_ = AdaptiveGate{};
    gate_accel_ = AdaptiveGate{};
    current_r_scale_left_ = 1.0;
    current_r_scale_right_ = 1.0;
    current_r_scale_gyro_ = 1.0;
    current_r_scale_accel_ = 1.0;
    qv_scale_ = 1.0;
    qw_scale_ = 1.0;
    common_slip_memory_ = 0.0;
    common_slip_memory_valid_ = false;
    common_slip_memory_last_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    covariance_valid_ = false;
    covariance_age_solves_ = 0;
    covariance_solve_counter_ = 0;
    covariance_runtime_ewma_ms_ = 0.0;
    covariance_retry_after_solve_ = 0;
    ++covariance_epoch_;  // invalidates in-flight snapshot on ALL horizon resets
    covariance_latest_sequence_ = 0;
    covariance_job_in_flight_ = false;
    solve_pending_ = false;
    have_last_accepted_endpoint_ = false;
    accepted_solution_count_ = 0;
    gate_state_last_.fill(0.0);

    // Do not carry stale median-filter samples across a horizon reset or a
    // ROS-time discontinuity. Keeping the published pose itself is still
    // allowed for ordinary sensor-gap recovery.
    output_v_target_history_.clear();
    output_w_target_history_.clear();

    if (reset_published_pose) {
      pub_state_initialized_ = false;
      output_shaper_initialized_ = false;
      pub_v_ = 0.0;
      pub_w_ = 0.0;
      pub_linear_accel_ = 0.0;
      pub_angular_accel_ = 0.0;
    }
  }

  void resetEstimatorForTimeJump()
  {
    // Events already queued belong to the OLD simulation-time epoch. If kept,
    // they could prevent new valid Gazebo events from entering the horizon.
    {
      std::lock_guard<std::mutex> lock(sensor_mutex_);
      sensor_event_queue_.clear();
      last_queue_depth_ = 0;
      last_deferred_future_events_ = 0;
    }
    resetHorizon(true);
    last_processed_sensor_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    ++backward_clock_jump_count_;
    have_last_ros_tick_ = false;
    RCLCPP_WARN(get_logger(), "ROS time moved backwards; MHE state was reset");
  }

  bool buildMeasurementFromEvent(
    const SensorEvent & event,
    double process_dt,
    Measurement & z)
  {
    z = Measurement{};
    z.dt = process_dt;
    z.stamp = event.stamp;

    if (event.type == SensorEventType::ODOM) {
      z.has_odom = true;
      z.odom_v = event.odom.twist.twist.linear.x;
      z.odom_w = event.odom.twist.twist.angular.z;

      if (!std::isfinite(z.odom_v) || !std::isfinite(z.odom_w) ||
          std::abs(z.odom_v) > max_abs_v_ || std::abs(z.odom_w) > max_abs_w_) {
        return false;
      }

      const double half_track = 0.5 * wheel_separation_;
      z.wheel_left = z.odom_v - half_track * z.odom_w;
      z.wheel_right = z.odom_v + half_track * z.odom_w;
      latest_odom_quiet_ =
        std::abs(z.odom_v) <= stationary_v_threshold_ &&
        std::abs(z.odom_w) <= stationary_w_threshold_;

      if (have_last_odom_for_accel_) {
        double odom_dt = (event.stamp - last_odom_stamp_for_accel_).seconds();
        odom_dt = std::clamp(odom_dt, 0.0005, 0.10);
        const double wheel_accel = (z.odom_v - last_odom_v_for_accel_) / odom_dt;
        wheel_accel_excitation_ = 0.80 * wheel_accel_excitation_ + 0.20 * wheel_accel;

        if (have_previous_corrected_accel_) {
          const double pair_age = std::abs(
            (event.stamp - previous_corrected_accel_stamp_).seconds());
          if (pair_age <= common_slip_accel_pair_max_age_) {
            z.has_common_slip_accel_pair = true;
            z.paired_imu_accel = previous_corrected_accel_;
            z.paired_wheel_accel = wheel_accel;
          }
        }
      }
      last_odom_v_for_accel_ = z.odom_v;
      last_odom_stamp_for_accel_ = event.stamp;
      have_last_odom_for_accel_ = true;
      last_measured_left_ = z.wheel_left;
      last_measured_right_ = z.wheel_right;
      return true;
    }

    z.has_gyro = true;
    z.has_accel = true;
    z.gyro_z = gyro_scale_ * event.imu.angular_velocity.z;

    if (!std::isfinite(z.gyro_z) || std::abs(z.gyro_z) > max_abs_gyro_) {
      z.has_gyro = false;
    }

    bool gravity_compensated = false;
    const double raw_accel = extractLongitudinalAcceleration(event.imu, gravity_compensated);
    accel_raw_longitudinal_ = raw_accel;
    accel_gravity_compensated_last_ = gravity_compensated;
    if (!std::isfinite(raw_accel) || std::abs(raw_accel) > max_abs_accel_) {
      z.has_accel = false;
    } else {
      double imu_dt = 1.0 / frequency_;
      if (have_last_imu_stamp_) {
        imu_dt = (event.stamp - last_imu_stamp_).seconds();
        imu_dt = std::clamp(imu_dt, 0.0005, 0.10);
      }
      last_imu_stamp_ = event.stamp;
      have_last_imu_stamp_ = true;

      if (!accel_filter_initialized_) {
        accel_filtered_ = raw_accel;
        accel_filter_initialized_ = true;
      } else {
        const double alpha = imu_dt / (accel_lpf_tau_ + imu_dt);
        accel_filtered_ += alpha * (raw_accel - accel_filtered_);
      }

      // Stationary auto-zero removes residual mounting/gravity projection even
      // when the IMU orientation is unavailable or gravity compensation is off.
      const double bg_estimate = states_.empty() ? 0.0 : states_.back()[BG];
      const bool gyro_quiet_for_zero =
        !z.has_gyro || std::abs(z.gyro_z - bg_estimate) <= stationary_gyro_threshold_;
      if (accel_auto_zero_enabled_ && latest_odom_quiet_ && gyro_quiet_for_zero &&
          std::abs(accel_filtered_) <= accel_auto_zero_max_abs_offset_) {
        accel_zero_counter_ = std::min(accel_zero_counter_ + 1, stationary_min_samples_);
        if (accel_zero_counter_ >= stationary_min_samples_) {
          if (!accel_zero_initialized_) {
            accel_zero_offset_ = accel_filtered_;
            accel_zero_initialized_ = true;
          } else {
            const double alpha = std::clamp(accel_auto_zero_alpha_, 0.0, 1.0);
            accel_zero_offset_ =
              (1.0 - alpha) * accel_zero_offset_ + alpha * accel_filtered_;
          }
        }
      } else {
        accel_zero_counter_ = 0;
      }

      double corrected_accel = accel_filtered_ -
        (accel_zero_initialized_ ? accel_zero_offset_ : 0.0);

      // Jerk limiting suppresses one-sample spikes without adding a long
      // low-pass lag to legitimate acceleration changes.
      if (have_previous_corrected_accel_) {
        double accel_dt = (event.stamp - previous_corrected_accel_stamp_).seconds();
        accel_dt = std::clamp(accel_dt, 0.0005, 0.10);
        const double max_delta = std::max(0.0, accel_max_jerk_) * accel_dt;
        corrected_accel = std::clamp(
          corrected_accel,
          previous_corrected_accel_ - max_delta,
          previous_corrected_accel_ + max_delta);
      }
      previous_corrected_accel_ = corrected_accel;
      previous_corrected_accel_stamp_ = event.stamp;
      have_previous_corrected_accel_ = true;
      accel_corrected_last_ = corrected_accel;
      z.accel_x = corrected_accel;
    }

    return z.has_gyro || z.has_accel;
  }

  void snapshotAdaptiveScales(Measurement & z) const
  {
    z.r_scale_left = current_r_scale_left_;
    z.r_scale_right = current_r_scale_right_;
    z.r_scale_gyro = current_r_scale_gyro_;
    z.r_scale_accel = current_r_scale_accel_;
    z.qv_scale = qv_scale_;
    z.qw_scale = qw_scale_;
  }

  void mergeMeasurement(Measurement & target, const Measurement & incoming)
  {
    if (incoming.has_odom) {
      target.has_odom = true;
      target.odom_v = incoming.odom_v;
      target.odom_w = incoming.odom_w;
      target.wheel_left = incoming.wheel_left;
      target.wheel_right = incoming.wheel_right;
      target.r_scale_left = incoming.r_scale_left;
      target.r_scale_right = incoming.r_scale_right;
    }
    if (incoming.has_gyro) {
      target.has_gyro = true;
      target.gyro_z = incoming.gyro_z;
      target.r_scale_gyro = incoming.r_scale_gyro;
    }
    if (incoming.has_accel) {
      target.has_accel = true;
      target.accel_x = incoming.accel_x;
      target.r_scale_accel = incoming.r_scale_accel;
    }
    if (incoming.has_common_slip_accel_pair) {
      target.has_common_slip_accel_pair = true;
      target.paired_imu_accel = incoming.paired_imu_accel;
      target.paired_wheel_accel = incoming.paired_wheel_accel;
    }
    // Q belongs to the transition ending at this state.  Keep the latest
    // adaptive snapshot if multiple near-synchronous events are merged.
    target.qv_scale = incoming.qv_scale;
    target.qw_scale = incoming.qw_scale;
    if (incoming.stamp > target.stamp) {
      target.stamp = incoming.stamp;
    }
  }

  size_t computeMarginalizeCount() const
  {
    if (states_.size() < 6 || measurements_.size() != states_.size()) {
      return 0;
    }

    const size_t min_states_to_keep = 5;
    const size_t max_states = static_cast<size_t>(std::max(5, max_horizon_states_));
    const double max_span = std::max(0.02, horizon_duration_sec_);

    size_t remove_count = 0;
    while (states_.size() - remove_count > min_states_to_keep) {
      const bool too_many = states_.size() - remove_count > max_states;
      const double span =
        (measurements_.back().stamp - measurements_[remove_count].stamp).seconds();
      const bool too_old = span > max_span;
      if (!too_many && !too_old) {
        break;
      }
      ++remove_count;
    }
    return remove_count;
  }

  void dropMarginalizedStates(size_t count)
  {
    count = std::min(count, states_.size());
    for (size_t i = 0; i < count; ++i) {
      states_.pop_front();
      measurements_.pop_front();
    }
  }

  void updateWarmStartFromMeasurement(std::array<double, NX> & x, const Measurement & z)
  {
    if (z.has_odom) {
      x[V] = 0.40 * x[V] + 0.60 * z.odom_v;
    }

    if (z.has_gyro && z.has_odom) {
      const double imu_w = z.gyro_z - x[BG];
      x[W] = 0.15 * x[W] + 0.20 * z.odom_w + 0.65 * imu_w;
    } else if (z.has_gyro) {
      const double imu_w = z.gyro_z - x[BG];
      x[W] = 0.25 * x[W] + 0.75 * imu_w;
    } else if (z.has_odom) {
      x[W] = 0.45 * x[W] + 0.55 * z.odom_w;
    }
  }

  bool initializeFromEvent(const SensorEvent & event, const Measurement & z)
  {
    if (require_odom_for_init_ && !z.has_odom) {
      return false;
    }
    if (!z.has_odom && !z.has_gyro) {
      return false;
    }

    std::array<double, NX> x{};
    if (z.has_odom) {
      x[PX] = event.odom.pose.pose.position.x;
      x[PY] = event.odom.pose.pose.position.y;
      x[YAW] = tf2::getYaw(event.odom.pose.pose.orientation);
      x[V] = z.odom_v;
      x[W] = z.odom_w;
    }
    if (z.has_gyro) {
      x[W] = z.gyro_z;
    }

    Measurement z0 = z;
    snapshotAdaptiveScales(z0);
    states_.push_back(x);
    measurements_.push_back(z0);
    arrival_state_ = x;
    last_measurement_stamp_ = event.stamp;
    last_processed_sensor_stamp_ = event.stamp;
    return true;
  }

  void processSensorEvent(const SensorEvent & event)
  {
    if (!states_.empty()) {
      const double delta = (event.stamp - last_measurement_stamp_).seconds();

      // A timestamp that is older but outside merge tolerance MUST NOT be
      // appended as a new forward state (even if within out-of-order tolerance).
      // That would silently turn a negative dt into +0.5 ms for the optimizer.
      if (delta < -out_of_order_tolerance_sec_ ||
          (delta < 0.0 && -delta > merge_tolerance_sec_)) {
        ++out_of_order_drop_count_;
        return;
      }

      if (std::abs(delta) <= merge_tolerance_sec_) {
        Measurement incoming;
        const double merge_dt = std::max(std::abs(delta), 0.0005);
        if (!buildMeasurementFromEvent(event, merge_dt, incoming)) {
          return;
        }

        // Evaluate health before the incoming factor can influence the state.
        updatePrefitSensorHealth(incoming);
        snapshotAdaptiveScales(incoming);
        if (!applyInnovationGating(incoming)) {
          return;
        }
        // Q adaptation uses the surviving sensors only.
        updateAdaptivePre(incoming);
        // Q snapshot is refreshed without overwriting gate-inflated R.
        incoming.qv_scale = qv_scale_;
        incoming.qw_scale = qw_scale_;

        if (incremental_graph_enabled_ && !states_.empty()) {
          graph_dirty_from_ = std::min(graph_dirty_from_, states_.size() - 1);
        }
        mergeMeasurement(measurements_.back(), incoming);
        auto & z = measurements_.back();
        z.stationary = detectStationary(z);
        updateSlipObservability(z);
        updateWarmStartFromMeasurement(states_.back(), z);
        if (z.stationary) {
          states_.back()[V] = 0.0;
          states_.back()[W] = 0.0;
        }

        solve_pending_ = states_.size() >= 5;
        if (!batch_solve_enabled_ && solve_pending_) {
          const size_t remove_count = computeMarginalizeCount();
          const bool marginalization_ok = solveMHE(remove_count);
          updatePostFitResiduals();
          if (marginalization_ok) {
            dropMarginalizedStates(remove_count);
          }
          solve_pending_ = false;
        }
        if (event.stamp > last_processed_sensor_stamp_) {
          last_processed_sensor_stamp_ = event.stamp;
        }
        if (event.stamp > last_measurement_stamp_) {
          last_measurement_stamp_ = event.stamp;
        }
        return;
      }

      if (delta > reset_gap_sec_) {
        ++gap_reset_count_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Sensor gap %.3f s exceeded %.3f s; resetting MHE horizon",
          delta, reset_gap_sec_);
        // Preserve the continuous published pose. Only the optimization horizon
        // is reset, avoiding a visible TF jump after a temporary sensor gap.
        resetHorizon(false);
      }
    }

    double dt = 1.0 / frequency_;
    if (!states_.empty()) {
      dt = (event.stamp - last_measurement_stamp_).seconds();
      dt = std::max(dt, 0.0005);
    }

    Measurement z;
    if (!buildMeasurementFromEvent(event, dt, z)) {
      return;
    }

    if (states_.empty()) {
      initializeFromEvent(event, z);
      return;
    }

    // Health/adaptive-R is based on the prior prediction, before warm-start
    // and before this measurement is added to the nonlinear problem.
    updatePrefitSensorHealth(z);
    snapshotAdaptiveScales(z);
    if (!applyInnovationGating(z)) {
      return;
    }
    updateAdaptivePre(z);
    z.qv_scale = qv_scale_;
    z.qw_scale = qw_scale_;

    z.stationary = detectStationary(z);
    updateSlipObservability(z);

    std::array<double, NX> x = states_.back();
    updateWarmStartFromMeasurement(x, z);

    if (z.stationary) {
      x[V] = 0.0;
      x[W] = 0.0;
    }

    if (exact_se2_motion_enabled_) {
      const auto arc = yaw_math::arcWithDerivatives(x[YAW], x[V], x[W], dt);
      x[PX] += arc.dx;
      x[PY] += arc.dy;
    } else {
      const double yaw_mid = x[YAW] + 0.5 * dt * x[W];
      x[PX] += dt * x[V] * std::cos(yaw_mid);
      x[PY] += dt * x[V] * std::sin(yaw_mid);
    }
    x[YAW] += dt * x[W];
    x[SL] *= slip_warm_start_decay_;
    x[SR] *= slip_warm_start_decay_;

    states_.push_back(x);
    measurements_.push_back(z);
    last_measurement_stamp_ = event.stamp;
    last_processed_sensor_stamp_ = event.stamp;

    solve_pending_ = states_.size() >= 5;
    if (!batch_solve_enabled_ && solve_pending_) {
      const size_t remove_count = computeMarginalizeCount();
      const bool marginalization_ok = solveMHE(remove_count);
      updatePostFitResiduals();
      if (marginalization_ok) {
        dropMarginalizedStates(remove_count);
      }
      solve_pending_ = false;
    }
  }

  void timerCallback()
  {
    const auto tick_start = std::chrono::steady_clock::now();
    if (have_last_wall_tick_) {
      last_wall_tick_period_ms_ = std::chrono::duration<double, std::milli>(
        tick_start - last_wall_tick_start_).count();
    }
    last_wall_tick_start_ = tick_start;
    have_last_wall_tick_ = true;
    // Fixed-rate publisher / propagator on ROS time. Sensor factors themselves
    // are asynchronous and use their own header timestamps.
    const rclcpp::Time ros_now = now();
    const bool had_prior_tick = have_last_ros_tick_;
    const double raw_dt = had_prior_tick ?
      (ros_now - last_ros_tick_).seconds() : 0.0;
    if (had_prior_tick && raw_dt < -1e-6) {
      resetEstimatorForTimeJump();
      last_ros_tick_ = ros_now;
      have_last_ros_tick_ = true;
      return;
    }
    if (had_prior_tick && raw_dt <= 1e-6) {
      return;  // /clock paused: do not duplicate old pose at a frozen stamp.
    }
    last_ros_tick_ = ros_now;
    have_last_ros_tick_ = true;

    acceptFastCovariance();
    auto events = drainSensorEvents(ros_now);
    last_processed_event_count_ = events.size();

    for (const auto & event : events) {
      processSensorEvent(event);
    }

    // Scalability optimization: process every asynchronous measurement first,
    // then perform one nonlinear solve for this output tick.  This keeps the
    // estimator at a deterministic <= output-rate solve cadence even when
    // several IMU/odom callbacks arrive together.
    if (batch_solve_enabled_ && solve_pending_ && states_.size() >= 5) {
      const size_t remove_count = computeMarginalizeCount();
      const bool marginalization_ok = solveMHE(remove_count);
      updatePostFitResiduals();
      if (marginalization_ok) {
        dropMarginalizedStates(remove_count);
      }
      solve_pending_ = false;
    }

    // Determine the integration step AFTER consuming sensor events; otherwise
    // the sensor-age test would miss a fresh event processed this tick.
    last_sim_tick_dt_sec_ = raw_dt;
    const bool have_sensor = last_processed_sensor_stamp_.nanoseconds() > 0;
    const double sensor_age = have_sensor ?
      (ros_now - last_processed_sensor_stamp_).seconds() : 0.0;
    const auto aligned_step = mhe_sensor_fusion::time_align::planStep(
      had_prior_tick, raw_dt, have_sensor, sensor_age,
      max_propagation_gap_sec_, sensor_timeout_sec_, future_tolerance_sec_);
    last_time_alignment_status_ = static_cast<int>(aligned_step.status);
    const bool hold_output = timestamp_aligned_output_enabled_ && aligned_step.stop_twist;
    if (timestamp_aligned_output_enabled_ &&
        aligned_step.status == mhe_sensor_fusion::time_align::StepStatus::ForwardGap) {
      ++forward_clock_gap_count_;
    }
    const double publish_dt = timestamp_aligned_output_enabled_ ?
      aligned_step.dt_sec :
      (had_prior_tick ? std::clamp(raw_dt, 0.001, 0.050) : 1.0 / frequency_);
    if (!states_.empty()) {
      publishState(ros_now, publish_dt, hold_output);
    }
    std_msgs::msg::Float64MultiArray time_alignment;
    // [0] exact ROS dt, [1] applied dt, [2] sensor age, [3] status enum,
    // [4] future events held in queue, [5] forward gap count,
    // [6] backward clock reset count, [7] yaw-feedback trust accepted,
    // [8] endpoint stamp age, [9] yaw correction, [10] gating rejects.
    time_alignment.data = {
      raw_dt, publish_dt, sensor_age,
      static_cast<double>(last_time_alignment_status_),
      static_cast<double>(last_deferred_future_events_),
      static_cast<double>(forward_clock_gap_count_),
      static_cast<double>(backward_clock_jump_count_),
      last_yaw_feedback_trusted_ ? 1.0 : 0.0,
      last_yaw_reference_age_sec_, last_yaw_feedback_correction_,
      static_cast<double>(yaw_feedback_trust_rejected_count_)};
    time_alignment_pub_->publish(time_alignment);
    last_callback_wall_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - tick_start).count();
    if (rt::deadlineMiss(last_callback_wall_ms_, rt_deadline_ms_)) {
      ++callback_deadline_misses_;
    }
    if (last_wall_tick_period_ms_ > 1.5 * rt_deadline_ms_) {
      ++tick_late_count_;
    }
    // Sampled after the publisher work, unlike /mhe/solver_health, whose
    // last_solver_time_ms is deliberately retained for backward compatibility.
    std_msgs::msg::Float64MultiArray profile;
    // [0] build, [1] Ceres, [2] covariance Jacobian snapshot (NOT worker), [3] marginalization,
    // [4] total MHE solve, [5] entire timer callback, [6] actual wall tick,
    // [7] solve deadline misses, [8] callback deadline misses,
    // [9] covariance deferrals, [10] covariance forced refreshes,
    // [11] total solve sequence, [12] tick lateness events, [13] rollbacks.
    profile.data = {
      last_graph_build_ms_, last_ceres_ms_, last_covariance_ms_,
      last_marginalization_ms_, last_solver_time_ms_,
      last_callback_wall_ms_, last_wall_tick_period_ms_,
      static_cast<double>(solver_deadline_misses_),
      static_cast<double>(callback_deadline_misses_),
      static_cast<double>(covariance_deferred_count_),
      static_cast<double>(covariance_forced_count_),
      static_cast<double>(total_solve_count_),
      static_cast<double>(tick_late_count_),
      static_cast<double>(solver_rollback_count_)};
    realtime_profile_pub_->publish(profile);

    // [0] submissions, [1] completed, [2] failed, [3] queue deferrals,
    // [4] discarded old generation, [5] age in accepted solves,
    // [6] worker compute_ms, [7] synchronous snapshot/Evaluate_ms,
    // [8] posterior valid, [9] last accepted covariance sequence.
    std_msgs::msg::Float64MultiArray worker_status;
    worker_status.data = {
      static_cast<double>(covariance_jobs_submitted_),
      static_cast<double>(covariance_jobs_completed_),
      static_cast<double>(covariance_failure_count_),
      static_cast<double>(covariance_deferred_count_),
      static_cast<double>(covariance_discarded_),
      static_cast<double>(covariance_age_solves_), covariance_worker_compute_ms_,
      last_covariance_ms_, covariance_valid_ ? 1.0 : 0.0,
      static_cast<double>(covariance_latest_sequence_),
      // [10] covariance method (0 legacy,1 scaled LLT,2 SVD,3 rejected),
      // [11] effective rank, [12] scaled reciprocal condition.
      static_cast<double>(covariance_last_method_),
      static_cast<double>(covariance_last_rank_), covariance_last_rcond_};
    covariance_worker_pub_->publish(worker_status);
  }

  bool marginalizePrefix(ceres::Problem & problem, size_t remove_count)
  {
    if (remove_count == 0) {
      return true;
    }
    if (states_.size() <= remove_count || measurements_.size() != states_.size()) {
      return false;
    }

    const size_t block_count = remove_count + 1;  // eliminated prefix + retained boundary
    const int total_dim = static_cast<int>(block_count * NX);
    const int elim_dim = static_cast<int>(remove_count * NX);
    const int keep_dim = NX;

    mhe_sensor_fusion::block_schur::Chain<NX> chain(static_cast<int>(block_count));
    Eigen::MatrixXd H;
    Eigen::VectorXd b;
    if (!block_schur_enabled_) {
      H = Eigen::MatrixXd::Zero(total_dim, total_dim);
      b = Eigen::VectorXd::Zero(total_dim);
    }

    std::vector<ceres::ResidualBlockId> residual_blocks;
    // A retained Ceres graph stores each factor by its latest state.  Prefix
    // marginalization can touch only (a) the old arrival prior, (b) unary
    // factors on eliminated states and (c) transitions ending no later than
    // the retained boundary.  Enumerate these O(remove_count) candidates
    // instead of scanning every factor in the full horizon.  Fall back to the
    // exhaustive scan for the legacy rebuild path or an invalid cache.
    const bool cached_prefix = incremental_graph_enabled_ &&
      graph_problem_.get() == &problem &&
      graph_entries_.size() == states_.size() && graph_prior_id_ != nullptr;
    if (cached_prefix) {
      residual_blocks = mhe_sensor_fusion::marginalization::prefixCandidates(
        graph_prior_id_, graph_entries_, remove_count);
    } else {
      problem.GetResidualBlocks(&residual_blocks);
    }
    last_marginal_candidate_count_ = residual_blocks.size();
    last_marginal_evaluation_count_ = 0;
    last_marginal_full_scan_ = !cached_prefix;

    auto localBlockIndex = [&](double * ptr) -> int {
      for (size_t j = 0; j < block_count; ++j) {
        if (states_[j].data() == ptr) {
          return static_cast<int>(j);
        }
      }
      return -1;
    };

    for (const auto residual_id : residual_blocks) {
      const auto * cost = problem.GetCostFunctionForResidualBlock(residual_id);
      if (!cost) {
        continue;
      }

      std::vector<double *> parameter_blocks;
      problem.GetParameterBlocksForResidualBlock(residual_id, &parameter_blocks);

      bool touches_eliminated = false;
      bool touches_outside_boundary = false;
      std::vector<int> local_indices(parameter_blocks.size(), -1);
      for (size_t i = 0; i < parameter_blocks.size(); ++i) {
        const int idx = localBlockIndex(parameter_blocks[i]);
        local_indices[i] = idx;
        if (idx >= 0 && idx < static_cast<int>(remove_count)) {
          touches_eliminated = true;
        } else if (idx < 0) {
          // A residual that touches an eliminated state and a state beyond the
          // retained boundary cannot be represented by a unary boundary prior.
          touches_outside_boundary = true;
        }
      }

      if (!touches_eliminated) {
        continue;
      }
      if (touches_outside_boundary) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Marginalization encountered a non-local factor; postponing prefix removal");
        return false;
      }

      ++last_marginal_evaluation_count_;

      const int nr = cost->num_residuals();
      std::vector<double> residual(static_cast<size_t>(nr), 0.0);
      std::vector<std::vector<double>> jac_storage(parameter_blocks.size());
      std::vector<double *> jac_ptrs(parameter_blocks.size(), nullptr);

      for (size_t i = 0; i < parameter_blocks.size(); ++i) {
        if (local_indices[i] >= 0) {
          jac_storage[i].resize(static_cast<size_t>(nr * NX));
          jac_ptrs[i] = jac_storage[i].data();
        }
      }

      double cost_value = 0.0;
      if (!problem.EvaluateResidualBlock(
          residual_id, true, &cost_value, residual.data(), jac_ptrs.data())) {
        return false;
      }

      Eigen::Map<const Eigen::VectorXd> r(residual.data(), nr);
      for (size_t i = 0; i < parameter_blocks.size(); ++i) {
        const int bi = local_indices[i];
        if (bi < 0) {
          continue;
        }
        const Eigen::Map<
          const Eigen::Matrix<double, Eigen::Dynamic, NX, Eigen::RowMajor>> Ji(
          jac_storage[i].data(), nr, NX);
        const int oi = bi * NX;
        const VecX gi = Ji.transpose() * r;
        if (block_schur_enabled_) {
          if (!chain.addRhs(bi, gi)) {return false;}
        } else {
          b.segment(oi, NX) += gi;
        }

        for (size_t j = i; j < parameter_blocks.size(); ++j) {
          const int bj = local_indices[j];
          if (bj < 0) {
            continue;
          }
          const Eigen::Map<
            const Eigen::Matrix<double, Eigen::Dynamic, NX, Eigen::RowMajor>> Jj(
            jac_storage[j].data(), nr, NX);
          const int oj = bj * NX;
          const Eigen::Matrix<double, NX, NX> Hij = Ji.transpose() * Jj;
          if (block_schur_enabled_) {
            if (!chain.add(bi, bj, Hij)) {return false;}
          } else {
            H.block(oi, oj, NX, NX).noalias() += Hij;
            if (oi != oj) {
              H.block(oj, oi, NX, NX).noalias() += Hij.transpose();
            }
          }
        }
      }
    }

    if (elim_dim <= 0 || total_dim != elim_dim + keep_dim) {
      return false;
    }

    MatX Hm = MatX::Zero();
    VecX bm = VecX::Zero();
    if (block_schur_enabled_) {
      if (!chain.eliminate(static_cast<int>(remove_count), Hm, bm)) {
        ++schur_numerical_fallback_count_;
        if (!chain.eliminateDense(static_cast<int>(remove_count), Hm, bm)) {
          return false;
        }
      }
    } else {
      Eigen::MatrixXd Haa = H.topLeftCorner(elim_dim, elim_dim);
      const Eigen::MatrixXd Hab = H.topRightCorner(elim_dim, keep_dim);
      const Eigen::MatrixXd Hba = H.bottomLeftCorner(keep_dim, elim_dim);
      const MatX Hbb = H.bottomRightCorner(keep_dim, keep_dim);
      const Eigen::VectorXd ba = b.head(elim_dim);
      const VecX bb = b.tail(keep_dim);
      Haa.diagonal().array() += 1e-8;
      Eigen::LDLT<Eigen::MatrixXd> ldlt(Haa);
      if (ldlt.info() != Eigen::Success || !ldlt.vectorD().allFinite() ||
          ldlt.vectorD().minCoeff() <= 0.0) {return false;}
      Hm = Hbb - Hba * ldlt.solve(Hab);
      bm = bb - Hba * ldlt.solve(ba);
    }
    Hm = 0.5 * (Hm + Hm.transpose());

    // Preserve the true information rank instead of injecting a fixed 1e-7
    // eigenvalue in physically unobservable slip/bias directions.
    if (rank_aware_prior_enabled_) {
      const auto prior = mhe_sensor_fusion::rank_aware::factorize<NX>(
        Hm, bm, prior_eigen_relative_cutoff_, 1e-7,
        prior_max_discarded_gradient_fraction_);
      if (!prior.valid) {
        ++rank_prior_reject_count_;
        return false;
      }
      Eigen::Map<const VecX> boundary(states_[remove_count].data());
      marginal_prior_.x_ref = boundary;
      marginal_prior_.sqrt_info = prior.sqrt_info;
      marginal_prior_.offset = prior.offset;
      marginal_prior_.valid = true;
      marginal_condition_ = prior.condition;
      marginal_prior_rank_ = prior.rank;
      marginal_prior_discarded_gradient_ = prior.discarded_gradient_norm;
      ++rank_prior_update_count_;
      last_marginalized_count_ = remove_count;
      return true;
    }

    Eigen::SelfAdjointEigenSolver<MatX> eigen_solver(Hm);
    if (eigen_solver.info() != Eigen::Success) {
      return false;
    }

    auto eigenvalues = eigen_solver.eigenvalues();
    const auto eigenvectors = eigen_solver.eigenvectors();
    constexpr double min_eigen = 1e-7;
    for (int i = 0; i < NX; ++i) {
      eigenvalues[i] = std::max(eigenvalues[i], min_eigen);
    }

    marginal_condition_ = eigenvalues.maxCoeff() /
                          std::max(eigenvalues.minCoeff(), min_eigen);
    Hm = eigenvectors * eigenvalues.asDiagonal() * eigenvectors.transpose();

    Eigen::LLT<MatX> llt(Hm);
    if (llt.info() != Eigen::Success) {
      return false;
    }

    const MatX L = llt.matrixL();
    const MatX A = L.transpose();
    const VecX c = L.triangularView<Eigen::Lower>().solve(bm);
    Eigen::Map<const VecX> boundary(states_[remove_count].data());

    marginal_prior_.x_ref = boundary;
    marginal_prior_.sqrt_info = A;
    marginal_prior_.offset = c;
    marginal_prior_.valid = true;
    last_marginalized_count_ = remove_count;
    return true;
  }

  void acceptFastCovariance()
  {
    fast_cov::Result result;
    if (!covariance_worker_.poll(result)) {return;}
    if (result.generation == covariance_epoch_) {
      covariance_job_in_flight_ = false;
    }
    if (result.generation != covariance_epoch_ ||
        result.sequence < covariance_latest_sequence_) {
      ++covariance_discarded_;
      return;
    }
    covariance_last_method_ = result.method;
    covariance_last_rank_ = result.effective_rank;
    covariance_last_rcond_ = result.reciprocal_condition;
    if (!result.valid || result.tail_size != NX ||
      result.covariance.size() != NX * NX) {
      // Reject ill-conditioned snapshots conservatively: do not keep an old
      // covariance marked valid after the current information loses rank.
      if (covariance_jacobi_scaled_enabled_) {covariance_valid_ = false;}
      covariance_worker_compute_ms_ = result.compute_ms;
      ++covariance_failure_count_;
      covariance_retry_after_solve_ = covariance_solve_counter_ +
        static_cast<size_t>(std::max(1, covariance_update_every_n_));
      return;
    }

    Eigen::Map<const Eigen::Matrix<double, NX, NX, Eigen::RowMajor>> raw(
      result.covariance.data());
    const MatX symmetric = 0.5 * (raw + raw.transpose()).eval();
    Eigen::SelfAdjointEigenSolver<MatX> eig(symmetric);
    if (eig.info() != Eigen::Success || !eig.eigenvalues().allFinite() ||
        eig.eigenvalues().minCoeff() <= 0.0) {
      ++covariance_failure_count_;
      return;
    }
    auto eigenvalues = eig.eigenvalues();
    // In the rank-aware scaled mode, clipping an enormous variance DOWN to
    // max_variance would falsely claim information we do not have. Prefer the
    // conservative invalid-covariance path until a better snapshot arrives.
    if (covariance_jacobi_scaled_enabled_ &&
        eigenvalues.maxCoeff() > covariance_max_variance_) {
      covariance_valid_ = false;
      ++covariance_failure_count_;
      return;
    }
    for (int i = 0; i < NX; ++i) {
      eigenvalues[i] = std::clamp(
        eigenvalues[i], covariance_min_variance_, covariance_max_variance_);
    }
    latest_covariance_ = eig.eigenvectors() * eigenvalues.asDiagonal() *
      eig.eigenvectors().transpose();
    if (!latest_covariance_.allFinite()) {
      ++covariance_failure_count_;
      return;
    }
    covariance_valid_ = true;
    // Do not mark an old background result as fresh: worker may complete
    // several solves after the snapshot was taken.
    covariance_age_solves_ = covariance_solve_counter_ >= result.sequence ?
      covariance_solve_counter_ - result.sequence : 0;
    covariance_latest_sequence_ = result.sequence;
    covariance_worker_compute_ms_ = result.compute_ms;
    covariance_last_method_ = result.method;
    covariance_last_rank_ = result.effective_rank;
    covariance_last_rcond_ = result.reciprocal_condition;
    ++covariance_jobs_completed_;
  }

  bool updatePosteriorCovariance(ceres::Problem & problem, double elapsed_ms)
  {
    if (!covariance_enabled_ || states_.empty()) {return false;}
    ++covariance_solve_counter_;
    ++covariance_age_solves_;
    if (covariance_age_solves_ > static_cast<size_t>(
        std::max(1, covariance_max_stale_solves_))) {
      // Revert to the conservative twist fallback when posterior gets stale.
      covariance_valid_ = false;
    }
    if (covariance_solve_counter_ < covariance_retry_after_solve_ ||
        static_cast<int>(states_.size()) > std::max(5, covariance_max_states_)) {
      return covariance_valid_;
    }

    if (!covariance_async_enabled_) {
      // Async worker is the default and required for low tail latency. Turning
      // it off deliberately uses the conservative fallback, not blocking SVD.
      return covariance_valid_;
    }
    const bool forced = covariance_age_solves_ >= static_cast<size_t>(
      std::max(1, covariance_max_age_solves_));
    const bool due = !covariance_valid_ || forced ||
      covariance_solve_counter_ % static_cast<size_t>(
        std::max(1, covariance_update_every_n_)) == 0;
    if (!due) {return covariance_valid_;}
    // Critical: do not compute a duplicate Jacobian while the worker owns a
    // previous snapshot. It runs on a separate thread and must never block
    // the 100Hz publisher.
    if (covariance_job_in_flight_) {
      ++covariance_deferred_count_;
      return covariance_valid_;
    }
    const double tick_elapsed = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - last_wall_tick_start_).count();
    if (covariance_defer_if_slow_ && !rt::canSnapshot(
      tick_elapsed, elapsed_ms, rt_deadline_ms_, covariance_snapshot_budget_ms_,
      covariance_rt_reserve_ms_, covariance_snapshot_ewma_ms_)) {
      ++covariance_deferred_count_;
      return covariance_valid_;
    }
    if (forced) {++covariance_forced_count_;}

    const auto started = std::chrono::steady_clock::now();
    ceres::Problem::EvaluateOptions options;
    options.apply_loss_function = true;
    options.parameter_blocks.reserve(states_.size());
    for (auto & state : states_) {
      options.parameter_blocks.push_back(state.data());
    }
    ceres::CRSMatrix jacobian;
    const bool ok = problem.Evaluate(options, nullptr, nullptr, nullptr, &jacobian);
    last_covariance_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    covariance_snapshot_ewma_ms_ = covariance_snapshot_ewma_ms_ <= 0.0 ?
      last_covariance_ms_ :
      0.85 * covariance_snapshot_ewma_ms_ + 0.15 * last_covariance_ms_;
    // This measures only snapshot creation; heavy selected inverse runs in
    // a dedicated thread. Remains a soft-RT operation, not hard WCET.
    if (!ok || jacobian.num_cols != static_cast<int>(states_.size()) * NX ||
      jacobian.num_rows <= 0) {
      ++covariance_failure_count_;
      covariance_retry_after_solve_ = covariance_solve_counter_ +
        static_cast<size_t>(std::max(1, covariance_update_every_n_));
      return covariance_valid_;
    }
    fast_cov::Snapshot snapshot;
    snapshot.generation = covariance_epoch_;
    snapshot.sequence = covariance_solve_counter_;
    snapshot.rows = jacobian.num_rows;
    snapshot.cols = jacobian.num_cols;
    snapshot.tail_size = NX;
    snapshot.jacobi_scaled_enabled = covariance_jacobi_scaled_enabled_;
    snapshot.svd_fallback_enabled = covariance_svd_fallback_enabled_;
    snapshot.minimum_rcond = covariance_minimum_rcond_;
    snapshot.svd_relative_cutoff = covariance_svd_relative_cutoff_;
    snapshot.maximum_svd_condition = covariance_maximum_svd_condition_;
    snapshot.row_offsets = std::move(jacobian.rows);
    snapshot.column_indices = std::move(jacobian.cols);
    snapshot.values = std::move(jacobian.values);
    if (!covariance_worker_.submit(std::move(snapshot))) {
      ++covariance_deferred_count_;
      return covariance_valid_;
    }
    ++covariance_jobs_submitted_;
    covariance_job_in_flight_ = true;
    return covariance_valid_;
  }

  struct GraphEntry
  {
    double * state{nullptr};
    std::vector<ceres::ResidualBlockId> local;
    std::vector<ceres::ResidualBlockId> transition;
  };

  void invalidateGraph()
  {
    graph_problem_.reset();
    graph_entries_.clear();
    graph_prior_id_ = nullptr;
    graph_dirty_from_ = static_cast<size_t>(-1);
    ++graph_reset_count_;
  }

  void addGraphBounds(size_t k)
  {
    graph_problem_->SetParameterLowerBound(states_[k].data(), V, -max_abs_v_);
    graph_problem_->SetParameterUpperBound(states_[k].data(), V, max_abs_v_);
    graph_problem_->SetParameterLowerBound(states_[k].data(), W, -max_abs_w_);
    graph_problem_->SetParameterUpperBound(states_[k].data(), W, max_abs_w_);
    graph_problem_->SetParameterLowerBound(states_[k].data(), BG, -0.8);
    graph_problem_->SetParameterUpperBound(states_[k].data(), BG, 0.8);
    graph_problem_->SetParameterLowerBound(states_[k].data(), BA, -3.0);
    graph_problem_->SetParameterUpperBound(states_[k].data(), BA, 3.0);
    graph_problem_->SetParameterLowerBound(states_[k].data(), SL, slip_left_min_);
    graph_problem_->SetParameterUpperBound(states_[k].data(), SL, slip_left_max_);
    graph_problem_->SetParameterLowerBound(states_[k].data(), SR, slip_right_min_);
    graph_problem_->SetParameterUpperBound(states_[k].data(), SR, slip_right_max_);

  }

  bool useGyroIncrement(size_t k) const
  {
    if (!gyro_increment_factor_enabled_ || k == 0 ||
      k >= measurements_.size() || !measurements_[k].has_gyro) {
      return false;
    }
    const double dt = (measurements_[k].stamp - measurements_[k - 1].stamp).seconds();
    // Do not turn a stale reading, a near-zero merge interval, or a gap into
    // an overconfident angular constraint. Use legacy unary gyro otherwise.
    return std::isfinite(dt) && dt >= 0.0005 && dt <= gyro_increment_max_dt_;
  }

  void addGraphLocal(size_t k)
  {
    auto & entry = graph_entries_[k];
    const auto & z = measurements_[k];
      // State-local slip observability: old factors are never retroactively
      // reweighted when the newest state changes from straight motion to turn.
      auto * slip_prior = new ceres::AutoDiffCostFunction<WheelSlipPriorCost, 2, NX>(
        new WheelSlipPriorCost(
          z.common_slip_reference,
          z.differential_slip_reference,
          std::max(z.sigma_common_slip, 1e-4),
          std::max(z.sigma_differential_slip, 1e-4)));
      entry.local.push_back(graph_problem_->AddResidualBlock(slip_prior, nullptr, states_[k].data()));

      if (z.has_odom) {
        ceres::CostFunction * wheel_cost = analytic_factors_enabled_ ?
          static_cast<ceres::CostFunction *>(new AnalyticWheelPairCost(
            z.wheel_left, z.wheel_right, wheel_separation_,
            sigma_wheel_left_base_ * z.r_scale_left,
            sigma_wheel_right_base_ * z.r_scale_right)) :
          static_cast<ceres::CostFunction *>(
            new ceres::AutoDiffCostFunction<WheelPairCost, 2, NX>(
              new WheelPairCost(z.wheel_left, z.wheel_right, wheel_separation_,
                sigma_wheel_left_base_ * z.r_scale_left,
                sigma_wheel_right_base_ * z.r_scale_right)));
        entry.local.push_back(graph_problem_->AddResidualBlock(
          wheel_cost, new ceres::HuberLoss(1.5), states_[k].data()));
      }

      if (z.has_gyro && !useGyroIncrement(k)) {
        auto * gyro_cost = new ceres::AutoDiffCostFunction<GyroCost, 1, NX>(
          new GyroCost(z.gyro_z, sigma_gyro_base_ * z.r_scale_gyro));
        entry.local.push_back(graph_problem_->AddResidualBlock(
          gyro_cost, new ceres::HuberLoss(1.5), states_[k].data()));
      }

      if (z.has_common_slip_accel_pair &&
          z.common_slip_observability >= common_slip_learn_observability_) {
        auto * common_slip_accel_cost =
          new ceres::AutoDiffCostFunction<CommonSlipAccelCost, 1, NX>(
          new CommonSlipAccelCost(
            z.paired_imu_accel,
            z.paired_wheel_accel,
            sigma_common_slip_accel_));
        entry.local.push_back(graph_problem_->AddResidualBlock(
          common_slip_accel_cost,
          new ceres::HuberLoss(1.5),
          states_[k].data()));
      }

      if (z.stationary) {
        auto * zero_motion = new ceres::AutoDiffCostFunction<ZeroMotionCost, 2, NX>(
          new ZeroMotionCost(stationary_sigma_v_, stationary_sigma_w_));
        entry.local.push_back(graph_problem_->AddResidualBlock(zero_motion, nullptr, states_[k].data()));
      }

  }

  void addGraphTransition(size_t k)
  {
    if (k == 0) {return;}
    auto & entry = graph_entries_[k];
    const auto & z = measurements_[k];
      double dt = (measurements_[k].stamp - measurements_[k - 1].stamp).seconds();
      dt = std::clamp(dt, 0.0005, 0.10);

      auto process_sigma = process_sigma_base_;
      const double nominal_dt = 1.0 / std::max(frequency_, 1.0);
      const double time_scale = std::sqrt(std::max(dt, 1e-6) / nominal_dt);
      for (auto & sigma : process_sigma) {
        sigma *= time_scale;
      }
      process_sigma[V] *= z.qv_scale;
      process_sigma[W] *= z.qw_scale;

      ceres::CostFunction * process_cost = analytic_factors_enabled_ ?
        static_cast<ceres::CostFunction *>(new AnalyticProcessCost(dt, process_sigma,
          exact_se2_motion_enabled_, so2_yaw_residual_enabled_)) :
        static_cast<ceres::CostFunction *>(
          new ceres::AutoDiffCostFunction<ProcessCost, NX, NX, NX>(
            new ProcessCost(dt, process_sigma,
              exact_se2_motion_enabled_, so2_yaw_residual_enabled_)));
      entry.transition.push_back(graph_problem_->AddResidualBlock(
        process_cost, nullptr, states_[k - 1].data(), states_[k].data()));

      if (useGyroIncrement(k)) {
        auto * gyro_inc_cost = new AnalyticGyroIncrementCost(
          z.gyro_z, dt, sigma_gyro_base_ * z.r_scale_gyro,
          gyro_increment_model_sigma_);
        entry.transition.push_back(graph_problem_->AddResidualBlock(
          gyro_inc_cost, new ceres::HuberLoss(1.5),
          states_[k - 1].data(), states_[k].data()));
      }

      if (z.has_accel) {
        auto * accel_cost = new ceres::AutoDiffCostFunction<AccelCost, 1, NX, NX>(
          new AccelCost(z.accel_x, dt, sigma_accel_base_ * z.r_scale_accel));
        entry.transition.push_back(graph_problem_->AddResidualBlock(
          accel_cost, new ceres::HuberLoss(std::max(accel_loss_delta_, 1e-3)),
          states_[k - 1].data(), states_[k].data()));
      }
  }

  void removeGraphBlocks(std::vector<ceres::ResidualBlockId> & ids)
  {
    for (auto id : ids) { graph_problem_->RemoveResidualBlock(id); }
    ids.clear();
  }

  ceres::Problem & prepareGraph()
  {
    const size_t n = states_.size();
    bool reset = !incremental_graph_enabled_ || !graph_problem_ ||
      graph_entries_.size() > n;
    if (!reset) {
      for (size_t k = 0; k < graph_entries_.size(); ++k) {
        if (graph_entries_[k].state != states_[k].data()) {
          reset = true;
          break;
        }
      }
    }
    if (reset) {
      invalidateGraph();
      ceres::Problem::Options graph_options;
      graph_options.enable_fast_removal = true;
      graph_problem_ = std::make_unique<ceres::Problem>(graph_options);
      for (size_t k = 0; k < n; ++k) {
        graph_problem_->AddParameterBlock(states_[k].data(), NX);
        graph_entries_.push_back(GraphEntry{states_[k].data(), {}, {}});
        addGraphBounds(k);
      }
      if (marginal_prior_.valid) {
        graph_prior_id_ = graph_problem_->AddResidualBlock(
          new MarginalPriorCost(marginal_prior_.x_ref,
            marginal_prior_.sqrt_info, marginal_prior_.offset,
            so2_yaw_residual_enabled_),
          nullptr, states_.front().data());
      } else {
        graph_prior_id_ = graph_problem_->AddResidualBlock(
          new ceres::AutoDiffCostFunction<ArrivalCost, NX, NX>(
            new ArrivalCost(arrival_state_, arrival_sigma_, so2_yaw_residual_enabled_)),
          nullptr, states_.front().data());
      }
      for (size_t k = 0; k < n; ++k) {
        addGraphLocal(k);
        addGraphTransition(k);
      }
      graph_dirty_from_ = static_cast<size_t>(-1);
      ++graph_full_build_count_;
      return *graph_problem_;
    }
    const size_t old_count = graph_entries_.size();
    for (size_t k = old_count; k < n; ++k) {
      graph_problem_->AddParameterBlock(states_[k].data(), NX);
      graph_entries_.push_back(GraphEntry{states_[k].data(), {}, {}});
      addGraphBounds(k);
    }
    // A merged IMU/odom event changes only the last existing state's factors.
    // Rebuild the dirty suffix, including its incoming transition (dt changes).
    const size_t from = std::min(graph_dirty_from_, old_count);
    for (size_t k = from; k < n; ++k) {
      removeGraphBlocks(graph_entries_[k].local);
      removeGraphBlocks(graph_entries_[k].transition);
      addGraphLocal(k);
      addGraphTransition(k);
      ++graph_updated_state_count_;
    }
    graph_dirty_from_ = static_cast<size_t>(-1);
    return *graph_problem_;
  }

  void advanceGraphAfterMarginalization(size_t remove_count)
  {
    if (!incremental_graph_enabled_ || !graph_problem_ || remove_count == 0) {return;}
    if (remove_count >= graph_entries_.size()) {
      invalidateGraph();
      return;
    }
    // Remove all touching factors *before* removing parameter blocks.
    if (graph_prior_id_) {
      graph_problem_->RemoveResidualBlock(graph_prior_id_);
      graph_prior_id_ = nullptr;
    }
    for (size_t k = 0; k < remove_count; ++k) {
      removeGraphBlocks(graph_entries_[k].local);
      removeGraphBlocks(graph_entries_[k].transition);
    }
    // The transition entering the retained boundary also touches old states.
    removeGraphBlocks(graph_entries_[remove_count].transition);
    for (size_t k = 0; k < remove_count; ++k) {
      graph_problem_->RemoveParameterBlock(graph_entries_[k].state);
    }
    graph_prior_id_ = graph_problem_->AddResidualBlock(
      new MarginalPriorCost(marginal_prior_.x_ref,
        marginal_prior_.sqrt_info, marginal_prior_.offset,
            so2_yaw_residual_enabled_),
      nullptr, graph_entries_[remove_count].state);
    graph_entries_.erase(graph_entries_.begin(), graph_entries_.begin() +
      static_cast<std::ptrdiff_t>(remove_count));
    ++graph_prune_count_;
  }

  bool solveMHE(size_t marginalize_count)
  {
    const auto solve_start = std::chrono::steady_clock::now();
    ++total_solve_count_;
    last_marginalized_count_ = 0;

    // A failed nonlinear step must never poison the next horizon or TF.
    const auto states_backup = states_;
    last_graph_build_ms_ = 0.0;
    last_ceres_ms_ = 0.0;
    last_covariance_ms_ = 0.0;
    last_marginalization_ms_ = 0.0;

    ceres::Problem & problem = prepareGraph();

    last_graph_build_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - solve_start).count();

    ceres::Solver::Options options;
    const bool use_dense_qr = !prefer_normal_cholesky_ &&
      static_cast<int>(states_.size()) <= std::max(5, dense_qr_max_states_);
    options.linear_solver_type =
      use_dense_qr ? ceres::DENSE_QR : ceres::DENSE_NORMAL_CHOLESKY;
    last_solver_mode_ = use_dense_qr ? 0 : 1;
    options.max_num_iterations = max_iterations_;
    options.num_threads = 1;
    options.minimizer_progress_to_stdout = false;
    options.function_tolerance = 1e-5;
    options.gradient_tolerance = 1e-6;
    options.parameter_tolerance = 1e-6;
    options.max_solver_time_in_seconds = solver_budget_ms_ / 1000.0;

    ceres::Solver::Summary summary;
    const auto ceres_start = std::chrono::steady_clock::now();
    ceres::Solve(options, &problem, &summary);

    auto solutionFinite = [&]() {
      for (const auto & state : states_) {
        for (const double value : state) {
          if (!std::isfinite(value)) {
            return false;
          }
        }
      }
      return true;
    };

    auto solutionFailureReason = [&]() -> safety::SolveRejectReason {
      // Do not trust a bogus endpoint from an unsuccessful nonlinear solve.
      const bool usable = summary.IsSolutionUsable();
      const bool finite = usable && solutionFinite();
      const bool cost_ok = usable && finite && safety::costAcceptable(
        summary.initial_cost, summary.final_cost, solver_cost_tolerance_);
      bool endpoint_ok = true;
      if (cost_ok && have_last_accepted_endpoint_ && !states_.empty()) {
        const double dt = (measurements_.back().stamp - last_accepted_stamp_).seconds();
        endpoint_ok = safety::endpointJumpAcceptable(
          states_.back()[V], states_.back()[W],
          last_accepted_v_, last_accepted_w_, dt,
          solver_max_endpoint_linear_accel_, solver_max_endpoint_angular_accel_,
          solver_linear_jump_margin_, solver_angular_jump_margin_);
      }
      return safety::classifySolveResult(usable, finite, cost_ok, endpoint_ok);
    };
    auto failure_reason = solutionFailureReason();

    // QR retries are reserved primarily for numerical/cost failures. In the
    // previous implementation EVERY rejection (including an endpoint jump)
    // triggered another complete 6-12 ms solve; that can delay a 100 Hz TF.
    // A retry now only receives the remainder of the original soft budget.
    const double elapsed_first_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - ceres_start).count();
    const auto qr_plan = safety::planQrFallback(
      failure_reason, use_dense_qr, elapsed_first_ms, solver_budget_ms_,
      qr_fallback_min_remaining_ms_, qr_fallback_on_endpoint_jump_);
    if (qr_plan.run) {
      for (size_t k = 0; k < states_.size() && k < states_backup.size(); ++k) {
        states_[k] = states_backup[k];
      }
      options.linear_solver_type = ceres::DENSE_QR;
      options.max_solver_time_in_seconds = qr_plan.remaining_ms / 1000.0;
      last_solver_mode_ = 2;  // normal-Cholesky -> time-budgeted QR fallback
      ++qr_fallback_attempt_count_;
      ceres::Solve(options, &problem, &summary);
      failure_reason = solutionFailureReason();
    } else if (failure_reason != safety::SolveRejectReason::None && !use_dense_qr) {
      ++qr_fallback_skip_count_;
    }
    last_rejection_reason_ = static_cast<int>(failure_reason);
    last_solution_usable_ = failure_reason == safety::SolveRejectReason::None;
    last_ceres_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - ceres_start).count();

    last_initial_cost_ = summary.initial_cost;
    last_final_cost_ = summary.final_cost;
    last_iteration_count_ = static_cast<double>(summary.iterations.size());

    if (!last_solution_usable_) {
      for (size_t k = 0; k < states_.size() && k < states_backup.size(); ++k) {
        states_[k] = states_backup[k];
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "MHE solve rejected; restored previous horizon state");
    }

    if (last_solution_usable_) {
      last_accepted_v_ = states_.back()[V];
      last_accepted_w_ = states_.back()[W];
      last_accepted_stamp_ = measurements_.back().stamp;
      have_last_accepted_endpoint_ = true;
      ++accepted_solution_count_;
      const double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - solve_start).count();
      updatePosteriorCovariance(problem, elapsed_ms);
    } else {
      ++solver_rollback_count_;
    }

    bool marginalization_ok = true;
    if (marginalize_count > 0 && last_solution_usable_) {
      const auto marginal_start = std::chrono::steady_clock::now();
      marginalization_ok = marginalizePrefix(problem, marginalize_count);
      if (marginalization_ok) {advanceGraphAfterMarginalization(marginalize_count);}
      last_marginalization_ms_ = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - marginal_start).count();
      if (!marginalization_ok) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "MHE prefix marginalization failed; horizon was NOT pruned");
      }
    } else if (marginalize_count > 0) {
      marginalization_ok = false;
    }

    const auto solve_end = std::chrono::steady_clock::now();
    last_solver_time_ms_ = std::chrono::duration<double, std::milli>(
      solve_end - solve_start).count();
    if (rt::deadlineMiss(last_solver_time_ms_, rt_deadline_ms_)) {
      ++solver_deadline_misses_;
    }
    if (!incremental_graph_enabled_) {invalidateGraph();}
    return marginalization_ok;
  }

  double median3OutputTarget(double target, std::deque<double> & history)
  {
    history.push_back(target);
    while (history.size() > 3) {
      history.pop_front();
    }

    if (!output_median3_enabled_ || history.size() < 3) {
      return target;
    }

    std::array<double, 3> values{history[0], history[1], history[2]};
    std::sort(values.begin(), values.end());
    return values[1];
  }

  double adaptiveOutputTau(
    double target, double value, double fixed_tau,
    double quiet_tau, double fast_tau, double transition) const
  {
    if (!output_adaptive_tau_enabled_) {
      return fixed_tau;
    }

    const double safe_transition = std::max(std::abs(transition), 1e-6);
    const double ratio = std::clamp(std::abs(target - value) / safe_transition, 0.0, 1.0);
    // Smoothstep avoids a slope discontinuity at the quiet/fast boundary.
    const double blend = ratio * ratio * (3.0 - 2.0 * ratio);
    return quiet_tau + (fast_tau - quiet_tau) * blend;
  }

  double shapeOutputChannel(
    double target, double dt, double tau,
    double max_accel, double max_decel, double max_jerk,
    double & value, double & acceleration)
  {
    if (dt <= 0.0 || !std::isfinite(target)) {
      return value;
    }

    const double safe_tau = std::max(tau, 0.0);
    const double alpha = safe_tau <= 1e-6 ? 1.0 : dt / (safe_tau + dt);
    const double filtered_target = value + alpha * (target - value);

    double desired_accel = (filtered_target - value) / dt;
    const bool slowing =
      (value * target < 0.0) || (std::abs(target) < std::abs(value));
    const double accel_limit = std::max(
      slowing ? max_decel : max_accel, 1e-6);
    desired_accel = std::clamp(desired_accel, -accel_limit, accel_limit);

    if (max_jerk > 1e-6) {
      const double max_da = max_jerk * dt;
      acceleration += std::clamp(
        desired_accel - acceleration, -max_da, max_da);
    } else {
      acceleration = desired_accel;
    }

    const double previous = value;
    value += acceleration * dt;

    // Do not let the smoothing dynamics overshoot the current target.
    if ((target - previous) * (target - value) <= 0.0) {
      value = target;
      acceleration = 0.0;
    }
    return value;
  }

  void publishState(const rclcpp::Time & stamp, double dt, bool force_hold)
  {
    if (states_.empty()) {
      return;
    }

    const auto & x = states_.back();
    const bool stationary_now =
      !measurements_.empty() && measurements_.back().stationary;

    double desired_v = x[V];
    double desired_w = x[W];

    bool sensor_stale = false;
    if (last_processed_sensor_stamp_.nanoseconds() > 0) {
      const double age = (stamp - last_processed_sensor_stamp_).seconds();
      sensor_stale = age > sensor_timeout_sec_;
      last_sensor_age_sec_ = std::max(0.0, age);
    }

    // Never dead-reckon indefinitely from a stale last velocity if both sensor
    // streams stop. Keep the last pose and publish zero twist instead.
    if (sensor_stale || force_hold) {
      desired_v = 0.0;
      desired_w = 0.0;
      // During a clock gap or stale data, do not claim a precisely known pose
      // merely because we froze its display. Increase uncertainty with time.
      if (force_hold && pub_state_initialized_) {
        const double lost_dt = std::max(0.0, last_sim_tick_dt_sec_);
        pub_pose_covariance_(0, 0) += 0.005 * lost_dt;
        pub_pose_covariance_(1, 1) += 0.005 * lost_dt;
        pub_pose_covariance_(2, 2) += 0.020 * lost_dt;
      }
    }

    if (stationary_now || std::abs(desired_v) < output_linear_deadband_) {
      desired_v = 0.0;
    }
    if (stationary_now || std::abs(desired_w) < output_angular_deadband_) {
      desired_w = 0.0;
    }

    if (!output_shaper_initialized_) {
      pub_v_ = desired_v;
      pub_w_ = desired_w;
      pub_linear_accel_ = 0.0;
      pub_angular_accel_ = 0.0;
      output_v_target_history_.clear();
      output_w_target_history_.clear();
      output_guarded_v_target_ = desired_v;
      output_guarded_w_target_ = desired_w;
      output_effective_linear_tau_ = output_linear_tau_;
      output_effective_angular_tau_ = output_angular_tau_;
      output_shaper_initialized_ = true;
    }

    if (sensor_stale || stationary_now || force_hold) {
      // Safety/stationary states snap exactly to zero; only normal moving
      // operation is smoothed. This preserves ZUPT and stale-sensor behavior.
      pub_v_ = 0.0;
      pub_w_ = 0.0;
      pub_linear_accel_ = 0.0;
      pub_angular_accel_ = 0.0;
      output_v_target_history_.clear();
      output_w_target_history_.clear();
      output_guarded_v_target_ = 0.0;
      output_guarded_w_target_ = 0.0;
    } else {
      // Apply smoothing per channel. In particular, no median/LPF/acceleration
      // shaping is allowed in the angular bypass. With master=false this is
      // bit-for-bit equivalent to the earlier unsmoothed branch.
      if (output_smoothing_enabled_ && output_smoothing_linear_enabled_) {
        output_guarded_v_target_ = median3OutputTarget(
          desired_v, output_v_target_history_);
        output_effective_linear_tau_ = adaptiveOutputTau(
          output_guarded_v_target_, pub_v_, output_linear_tau_,
          output_linear_quiet_tau_, output_linear_fast_tau_,
          output_linear_transition_);
        shapeOutputChannel(
          output_guarded_v_target_, dt, output_effective_linear_tau_,
          output_max_linear_accel_, output_max_linear_decel_,
          output_max_linear_jerk_, pub_v_, pub_linear_accel_);
      } else {
        output_v_target_history_.clear();
        output_guarded_v_target_ = desired_v;
        output_effective_linear_tau_ = 0.0;
        pub_v_ = desired_v;
        pub_linear_accel_ = 0.0;
      }

      if (output_smoothing_enabled_ && output_smoothing_angular_enabled_) {
        output_guarded_w_target_ = median3OutputTarget(
          desired_w, output_w_target_history_);
        output_effective_angular_tau_ = adaptiveOutputTau(
          output_guarded_w_target_, pub_w_, output_angular_tau_,
          output_angular_quiet_tau_, output_angular_fast_tau_,
          output_angular_transition_);
        shapeOutputChannel(
          output_guarded_w_target_, dt, output_effective_angular_tau_,
          output_max_angular_accel_, output_max_angular_decel_,
          output_max_angular_jerk_, pub_w_, pub_angular_accel_);
      } else {
        output_w_target_history_.clear();
        output_guarded_w_target_ = desired_w;
        output_effective_angular_tau_ = 0.0;
        pub_w_ = desired_w;
        pub_angular_accel_ = 0.0;
      }
    }

    const double output_v = pub_v_;
    double output_w = pub_w_;

    if (!pub_state_initialized_) {
      pub_x_ = x[PX];
      pub_y_ = x[PY];
      pub_yaw_ = std::atan2(std::sin(x[YAW]), std::cos(x[YAW]));
      pub_state_initialized_ = true;
      pub_pose_covariance_ = Eigen::Matrix3d::Zero();
      pub_pose_covariance_.diagonal() << 0.0004, 0.0004, 0.000625;
    } else if (dt > 0.0 && !stationary_now) {
      // Publish a continuous odometry trajectory from the optimized velocity.
      // This prevents tiny endpoint changes from the nonlinear solve from
      // appearing as backward/sideways jumps in odom->base_footprint.
      // F and G are derivatives of the SAME integration law used for pose.
      // This keeps yaw/translation covariance consistent through a turn.
      yaw_math::ArcDerivatives arc;
      if (exact_se2_motion_enabled_) {
        arc = yaw_math::arcWithDerivatives(pub_yaw_, output_v, output_w, dt);
      } else {
        const double yaw_mid = pub_yaw_ + 0.5 * output_w * dt;
        arc.dx = output_v * std::cos(yaw_mid) * dt;
        arc.dy = output_v * std::sin(yaw_mid) * dt;
        arc.dx_dyaw = -arc.dy;
        arc.dy_dyaw = arc.dx;
        arc.dx_dv = std::cos(yaw_mid) * dt;
        arc.dy_dv = std::sin(yaw_mid) * dt;
        arc.dx_dw = -0.5 * output_v * std::sin(yaw_mid) * dt * dt;
        arc.dy_dw = 0.5 * output_v * std::cos(yaw_mid) * dt * dt;
      }
      pub_x_ += arc.dx;
      pub_y_ += arc.dy;
      pub_yaw_ = yaw_math::wrapDifference(pub_yaw_ + output_w * dt);

      // The published pose is an INTEGRATED output trajectory, not the Ceres
      // window endpoint. Propagate its own uncertainty rather than using
      // the unrelated endpoint pose covariance as though it were output.
      Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
      F(0, 2) = arc.dx_dyaw;
      F(1, 2) = arc.dy_dyaw;
      Eigen::Matrix<double, 3, 2> G;
      G << arc.dx_dv, arc.dx_dw,
           arc.dy_dv, arc.dy_dw,
           0.0, dt;
      Eigen::Matrix2d velocity_noise = Eigen::Matrix2d::Zero();
      const double uncertainty_scale = 1.0 / std::max(confidence_, 0.10);
      velocity_noise(0, 0) = std::max(
        covariance_valid_ ? latest_covariance_(V, V) : 0.0, 0.03 * 0.03) * uncertainty_scale;
      velocity_noise(1, 1) = std::max(
        covariance_valid_ ? latest_covariance_(W, W) : 0.0, 0.03 * 0.03) * uncertainty_scale;
      pub_pose_covariance_ = F * pub_pose_covariance_ * F.transpose() +
        G * velocity_noise * G.transpose();
      // Extra per-second process-floor accounts for slip/bias/systematic drift
      // that velocity-sample covariance alone cannot capture.
      pub_pose_covariance_(0, 0) += 1e-5 * dt * uncertainty_scale;
      pub_pose_covariance_(1, 1) += 1e-5 * dt * uncertainty_scale;
      pub_pose_covariance_(2, 2) += 1e-5 * dt * uncertainty_scale;
      pub_pose_covariance_ = 0.5 * (
        pub_pose_covariance_ + pub_pose_covariance_.transpose()).eval();
    }

    // Optional bounded yaw reconciliation: the optimized endpoint is at the
    // *last sensor timestamp*, not the publication timestamp. Extrapolate to
    // the latter, reject stale/large discrepancies, and apply a rate-capped
    // correction. Never run this on rejected solves, missing sensors or ZUPT.
    last_yaw_feedback_correction_ = 0.0;
    last_yaw_feedback_error_ = 0.0;
    last_yaw_feedback_trusted_ = false;
    last_yaw_reference_age_sec_ = 0.0;
    if (yaw_feedback_policy_.enabled && !force_hold && !sensor_stale &&
        !stationary_now && accepted_solution_count_ > 0 &&
        last_solution_usable_ && !measurements_.empty() &&
        last_accepted_stamp_.nanoseconds() > 0) {
      // The Ceres endpoint is defined at measurements_.back().stamp, NOT at
      // ros_now and NOT at an arbitrary newest arrival from the event queue.
      const double reference_age = (stamp - measurements_.back().stamp).seconds();
      const double reference_skew =
        (measurements_.back().stamp - last_accepted_stamp_).seconds();
      last_yaw_reference_age_sec_ = reference_age;
      last_yaw_feedback_trusted_ = mhe_sensor_fusion::trustworthyYawEndpoint(
        true, reference_skew, confidence_, raw_nis_gyro_, covariance_valid_,
        covariance_valid_ ? latest_covariance_(YAW, YAW) : 0.0,
        static_cast<unsigned int>(covariance_age_solves_), yaw_feedback_policy_);
      if (!last_yaw_feedback_trusted_) {
        ++yaw_feedback_trust_rejected_count_;
      }
      const auto correction = last_yaw_feedback_trusted_ ?
        mhe_sensor_fusion::boundedYawFeedback(
          pub_yaw_, x[YAW], x[W], reference_age, dt, yaw_feedback_policy_) :
        mhe_sensor_fusion::YawFeedbackResult{};
      if (correction.applied) {
        pub_yaw_ = mhe_sensor_fusion::wrapYaw(pub_yaw_ + correction.step_rad);
        // Pose derivative includes the bounded correction; twist must agree.
        if (dt > 0.0) {
          output_w += correction.step_rad / dt;
        }
        last_yaw_feedback_correction_ = correction.step_rad;
        last_yaw_feedback_error_ = correction.error_rad;
        ++yaw_feedback_applied_count_;
        // A bounded correction has model uncertainty. Inflate yaw variance
        // instead of falsely reporting identical covariance after correction.
        pub_pose_covariance_(2, 2) += correction.step_rad * correction.step_rad;
      }
    }

    last_se2_correction_distance_ = 0.0;
    last_se2_correction_yaw_ = 0.0;
    last_se2_correction_trusted_ = false;
    if (se2_correction_policy_.enabled && !force_hold && !sensor_stale &&
        !stationary_now && dt > 0.0 && accepted_solution_count_ > 0 &&
        last_solution_usable_ && !measurements_.empty() &&
        last_accepted_stamp_.nanoseconds() > 0) {
      const double age = (stamp - measurements_.back().stamp).seconds();
      const double skew = (measurements_.back().stamp - last_accepted_stamp_).seconds();
      last_se2_correction_trusted_ = mhe_sensor_fusion::trustworthyYawEndpoint(
        true, skew, confidence_, raw_nis_gyro_, covariance_valid_,
        covariance_valid_ ? latest_covariance_(YAW, YAW) : 0.0,
        static_cast<unsigned int>(covariance_age_solves_), yaw_feedback_policy_);
      if (last_se2_correction_trusted_ && age >= 0.0 &&
          age <= se2_correction_policy_.max_age_sec) {
        const auto reference = se2_correct::predict(
          {x[PX], x[PY], x[YAW]}, x[V], x[W], age);
        const auto correction = se2_correct::reconcile(
          {pub_x_, pub_y_, pub_yaw_}, reference, dt, age, skew,
          true, se2_correction_policy_);
        if (correction.applied) {
          pub_x_ = correction.pose.x;
          pub_y_ = correction.pose.y;
          pub_yaw_ = correction.pose.yaw;
          last_se2_correction_distance_ = correction.applied_distance;
          last_se2_correction_yaw_ = correction.applied_yaw;
          ++se2_correction_applied_count_;
          // Conservative uncertainty increase for an approximate correction;
          // do not pretend the fitted endpoint is exact ground truth.
          pub_pose_covariance_(0, 0) += correction.applied_distance * correction.applied_distance;
          pub_pose_covariance_(1, 1) += correction.applied_distance * correction.applied_distance;
          pub_pose_covariance_(2, 2) += correction.applied_yaw * correction.applied_yaw;
        }
      }
    }

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pub_yaw_);

    nav_msgs::msg::Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = odom_frame_;
    msg.child_frame_id = base_frame_;
    msg.pose.pose.position.x = pub_x_;
    msg.pose.pose.position.y = pub_y_;
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation = tf2::toMsg(q);
    msg.twist.twist.linear.x = output_v;
    msg.twist.twist.angular.z = output_w;

    const double output_confidence = sensor_stale ? 0.25 * confidence_ : confidence_;
    const double confidence_safe = std::max(output_confidence, 0.10);
    const double cov_scale = 1.0 / confidence_safe;
    const double unobserved_variance = 1e6;
    msg.pose.covariance[14] = unobserved_variance;  // z
    msg.pose.covariance[21] = unobserved_variance;  // roll
    msg.pose.covariance[28] = unobserved_variance;  // pitch
    msg.twist.covariance[7] = unobserved_variance;   // vy
    msg.twist.covariance[14] = unobserved_variance;  // vz
    msg.twist.covariance[21] = unobserved_variance;  // wx
    msg.twist.covariance[28] = unobserved_variance;  // wy

    if (covariance_valid_) {
      // Ceres posterior block for the newest MHE state. Inflate gently by
      // confidence and by covariance age when it is intentionally evaluated
      // at a lower rate than the optimizer.
      const double posterior_scale = cov_scale *
        (1.0 + 0.05 * static_cast<double>(covariance_age_solves_));

      msg.twist.covariance[0] = latest_covariance_(V, V) * posterior_scale;
      msg.twist.covariance[5] = latest_covariance_(V, W) * posterior_scale;
      msg.twist.covariance[30] = latest_covariance_(W, V) * posterior_scale;
      msg.twist.covariance[35] = latest_covariance_(W, W) * posterior_scale;
    } else {
      // Conservative fallback until the first posterior covariance is ready.
      msg.twist.covariance[0] = std::pow(
        0.5 * (sigma_wheel_left_base_ * current_r_scale_left_ +
               sigma_wheel_right_base_ * current_r_scale_right_), 2);
      msg.twist.covariance[35] = std::pow(
        sigma_gyro_base_ * current_r_scale_gyro_, 2);
    }
    const double pose_inflation = 1.0 / std::max(output_confidence, 0.10);
    // Calibration is OPT-IN after independent truth/holdout verification.
    // Never update these factors from the same trajectory at runtime.
    Eigen::Matrix3d published_pose_covariance = pub_pose_covariance_ * pose_inflation;
    if (pose_covariance_calibration_enabled_) {
      published_pose_covariance = mhe_sensor_fusion::calibratePublishedPoseCovariance(
        published_pose_covariance, pose_covariance_std_scales_);
    }
    msg.pose.covariance[0] = published_pose_covariance(0, 0);
    msg.pose.covariance[1] = published_pose_covariance(0, 1);
    msg.pose.covariance[5] = published_pose_covariance(0, 2);
    msg.pose.covariance[6] = published_pose_covariance(1, 0);
    msg.pose.covariance[7] = published_pose_covariance(1, 1);
    msg.pose.covariance[11] = published_pose_covariance(1, 2);
    msg.pose.covariance[30] = published_pose_covariance(2, 0);
    msg.pose.covariance[31] = published_pose_covariance(2, 1);
    msg.pose.covariance[35] = published_pose_covariance(2, 2);
    odom_pub_->publish(msg);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp + rclcpp::Duration::from_seconds(transform_time_offset_);
      tf.header.frame_id = odom_frame_;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = pub_x_;
      tf.transform.translation.y = pub_y_;
      tf.transform.translation.z = 0.0;
      tf.transform.rotation = tf2::toMsg(q);
      tf_broadcaster_->sendTransform(tf);
    }

    std_msgs::msg::Float64 conf;
    conf.data = output_confidence;
    confidence_pub_->publish(conf);

    std_msgs::msg::Float64MultiArray gating_status;
    gating_status.data = {
      raw_nis_wheel_left_, raw_nis_wheel_right_, raw_nis_gyro_, raw_nis_accel_,
      gate_state_last_[0], gate_state_last_[1],
      gate_state_last_[2], gate_state_last_[3],
      static_cast<double>(gate_reject_counts_[0]),
      static_cast<double>(gate_reject_counts_[1]),
      static_cast<double>(gate_reject_counts_[2]),
      static_cast<double>(gate_reject_counts_[3])};
    gating_status_pub_->publish(gating_status);

    std_msgs::msg::Float64MultiArray solver_health;
    solver_health.data = {
      last_solution_usable_ ? 1.0 : 0.0,
      last_initial_cost_, last_final_cost_,
      last_initial_cost_ > 1e-12 ?
        (last_initial_cost_ - last_final_cost_) / last_initial_cost_ : 0.0,
      last_iteration_count_, last_solver_time_ms_,
      static_cast<double>(solver_rollback_count_),
      static_cast<double>(last_solver_mode_),
      pub_pose_covariance_(0, 0), pub_pose_covariance_(1, 1),
      pub_pose_covariance_(2, 2),
      // Appended without changing the 11 original solver_health fields:
      // [11] reject reason (0 good, 1 numeric, 2 cost, 3 endpoint)
      // [12] cumulative QR retries, [13] cumulative skipped QR retries.
      static_cast<double>(last_rejection_reason_),
      static_cast<double>(qr_fallback_attempt_count_),
      static_cast<double>(qr_fallback_skip_count_)};
    solver_health_pub_->publish(solver_health);
    // [0] incremental graph enabled, [1] block Schur enabled,
    // [2] full graph builds, [3] appended/refreshed states,
    // [4] successful graph prefix prunes, [5] resets,
    // [6] live parameter blocks, [7] live residual blocks,
    // [8] current graph build ms, [9] current marginalization ms,
    // [10] cumulative block-to-dense Schur numerical fallback.
    // [11] marginal candidates visited, [12] factors evaluated,
    // [13] fallback-to-full-scan flag (0 = cached prefix selection).
    std_msgs::msg::Float64MultiArray graph_status;
    graph_status.data = {
      incremental_graph_enabled_ ? 1.0 : 0.0,
      block_schur_enabled_ ? 1.0 : 0.0,
      static_cast<double>(graph_full_build_count_),
      static_cast<double>(graph_updated_state_count_),
      static_cast<double>(graph_prune_count_),
      static_cast<double>(graph_reset_count_),
      graph_problem_ ? static_cast<double>(graph_problem_->NumParameterBlocks()) : 0.0,
      graph_problem_ ? static_cast<double>(graph_problem_->NumResidualBlocks()) : 0.0,
      last_graph_build_ms_, last_marginalization_ms_,
      static_cast<double>(schur_numerical_fallback_count_),
      static_cast<double>(last_marginal_candidate_count_),
      static_cast<double>(last_marginal_evaluation_count_),
      last_marginal_full_scan_ ? 1.0 : 0.0,
      // [14] rank-aware prior enabled, [15] retained rank (0..9),
      // [16] discarded gradient norm, [17] cumulative rank-aware updates,
      // [18] invalid indefinite-information rejections.
      rank_aware_prior_enabled_ ? 1.0 : 0.0,
      static_cast<double>(marginal_prior_rank_),
      marginal_prior_discarded_gradient_,
      static_cast<double>(rank_prior_update_count_),
      static_cast<double>(rank_prior_reject_count_),
      // [19] gyro increment mode, [20] active in-horizon gyro increment factors,
      // [21] max interval s, [22] quadrature/model sigma rad/s.
      gyro_increment_factor_enabled_ ? 1.0 : 0.0,
      static_cast<double>([&]() {
        size_t count = 0;
        for (size_t k = 1; k < measurements_.size(); ++k) {
          if (useGyroIncrement(k)) {++count;}
        }
        return count;
      }()),
      gyro_increment_max_dt_, gyro_increment_model_sigma_};
    graph_status_pub_->publish(graph_status);

    std_msgs::msg::Float64MultiArray bias;
    bias.data = {x[BG], x[BA]};
    bias_pub_->publish(bias);

    std_msgs::msg::Float64MultiArray innovation;
    innovation.data = {
      prefit_e_left_, prefit_e_right_, prefit_e_gyro_, prefit_e_accel_,
      current_r_scale_left_, current_r_scale_right_,
      current_r_scale_gyro_, current_r_scale_accel_,
      qv_scale_, qw_scale_,
      stationary_now ? 1.0 : 0.0,
      last_solution_usable_ ? 1.0 : 0.0
    };
    innovation_pub_->publish(innovation);

    std_msgs::msg::Float64MultiArray postfit;
    postfit.data = {last_e_left_, last_e_right_, last_e_gyro_, last_e_accel_};
    postfit_residual_pub_->publish(postfit);

    std_msgs::msg::Float64 runtime;
    runtime.data = last_solver_time_ms_;
    solve_time_pub_->publish(runtime);

    std_msgs::msg::Float64 condition;
    condition.data = marginal_condition_;
    marginal_condition_pub_->publish(condition);

    std_msgs::msg::Float64MultiArray slip;
    const double half_track = 0.5 * wheel_separation_;
    const double true_left = x[V] - half_track * x[W];
    const double true_right = x[V] + half_track * x[W];
    const double common_slip = 0.5 * (x[SL] + x[SR]);
    const double differential_slip = 0.5 * (x[SR] - x[SL]);

    const double measured_left = last_measured_left_;
    const double measured_right = last_measured_right_;

    slip.data = {
      x[SL], x[SR],
      common_slip, differential_slip,
      measured_left, measured_right,
      true_left, true_right,
      common_slip_observability_, differential_slip_observability_,
      current_r_scale_left_, current_r_scale_right_,
      common_slip_memory_valid_ ? common_slip_memory_ : 0.0,
      measurements_.empty() ? sigma_slip_common_unobservable_ :
        measurements_.back().sigma_common_slip
    };
    wheel_slip_pub_->publish(slip);

    std_msgs::msg::Float64MultiArray covariance_diag;
    covariance_diag.data = {
      covariance_valid_ ? 1.0 : 0.0,
      covariance_valid_ ? latest_covariance_(PX, PX) : 0.0,
      covariance_valid_ ? latest_covariance_(PY, PY) : 0.0,
      covariance_valid_ ? latest_covariance_(YAW, YAW) : 0.0,
      covariance_valid_ ? latest_covariance_(V, V) : 0.0,
      covariance_valid_ ? latest_covariance_(W, W) : 0.0,
      covariance_valid_ ? latest_covariance_(PX, PY) : 0.0,
      covariance_valid_ ? latest_covariance_(V, W) : 0.0,
      static_cast<double>(covariance_age_solves_),
      static_cast<double>(covariance_failure_count_)
    };
    covariance_pub_->publish(covariance_diag);

    std_msgs::msg::Float64MultiArray accel_status;
    accel_status.data = {
      accel_raw_longitudinal_,
      accel_filtered_,
      accel_zero_offset_,
      accel_corrected_last_,
      accel_gravity_compensated_last_ ? 1.0 : 0.0,
      accel_zero_initialized_ ? 1.0 : 0.0
    };
    accel_status_pub_->publish(accel_status);

    std_msgs::msg::Float64MultiArray output_status;
    output_status.data = {
      x[V], x[W],
      desired_v, desired_w,
      output_v, output_w,
      pub_linear_accel_, pub_angular_accel_,
      common_slip, common_slip_observability_,
      output_smoothing_enabled_ ? 1.0 : 0.0,
      stationary_now ? 1.0 : 0.0,
      output_guarded_v_target_, output_guarded_w_target_,
      output_effective_linear_tau_, output_effective_angular_tau_,
      output_smoothing_linear_enabled_ ? 1.0 : 0.0,
      output_smoothing_angular_enabled_ ? 1.0 : 0.0,
      last_yaw_feedback_error_, last_yaw_feedback_correction_,
      static_cast<double>(yaw_feedback_applied_count_),
      yaw_feedback_policy_.enabled ? 1.0 : 0.0,
      // Appended to preserve legacy positions 0..23.
      timestamp_aligned_output_enabled_ ? 1.0 : 0.0,
      last_yaw_feedback_trusted_ ? 1.0 : 0.0,
      static_cast<double>(last_time_alignment_status_),
      last_yaw_reference_age_sec_,
      // [26..29] bounded SE(2) correction telemetry.
      se2_correction_policy_.enabled ? 1.0 : 0.0,
      last_se2_correction_distance_, last_se2_correction_yaw_,
      static_cast<double>(se2_correction_applied_count_)
    };
    output_status_pub_->publish(output_status);

    std_msgs::msg::Float64MultiArray raw_nis;
    raw_nis.data = {
      raw_nis_wheel_left_, raw_nis_wheel_right_, raw_nis_gyro_, raw_nis_accel_
    };
    raw_nis_pub_->publish(raw_nis);

    std_msgs::msg::Float64MultiArray health;
    health.data = {
      nis_wheel_left_, nis_wheel_right_, nis_gyro_, nis_accel_,
      current_r_scale_left_, current_r_scale_right_,
      current_r_scale_gyro_, current_r_scale_accel_,
      gate_left_.degraded ? 1.0 : 0.0,
      gate_right_.degraded ? 1.0 : 0.0,
      gate_gyro_.degraded ? 1.0 : 0.0,
      gate_accel_.degraded ? 1.0 : 0.0
    };
    sensor_health_pub_->publish(health);

    double horizon_span = 0.0;
    if (measurements_.size() >= 2) {
      horizon_span = (measurements_.back().stamp - measurements_.front().stamp).seconds();
    }
    std_msgs::msg::Float64MultiArray timing;
    timing.data = {
      static_cast<double>(last_processed_event_count_),
      static_cast<double>(last_queue_depth_),
      static_cast<double>(queue_drop_count_),
      static_cast<double>(out_of_order_drop_count_),
      static_cast<double>(gap_reset_count_),
      last_sensor_age_sec_,
      horizon_span,
      static_cast<double>(states_.size()),
      static_cast<double>(last_marginalized_count_),
      static_cast<double>(last_solver_mode_),
      static_cast<double>(covariance_age_solves_),
      static_cast<double>(total_solve_count_)
    };
    timing_pub_->publish(timing);
  }


  std::mutex sensor_mutex_;
  std::deque<SensorEvent> sensor_event_queue_;
  uint64_t next_event_sequence_{0};
  size_t queue_drop_count_{0};
  size_t out_of_order_drop_count_{0};
  size_t gap_reset_count_{0};
  size_t last_queue_depth_{0};
  size_t last_deferred_future_events_{0};
  uint64_t forward_clock_gap_count_{0};
  uint64_t backward_clock_jump_count_{0};
  double last_sim_tick_dt_sec_{0.0};
  int last_time_alignment_status_{1};
  bool last_yaw_feedback_trusted_{false};
  double last_yaw_reference_age_sec_{0.0};
  uint64_t yaw_feedback_trust_rejected_count_{0};
  size_t last_processed_event_count_{0};
  size_t last_marginalized_count_{0};
  size_t total_solve_count_{0};
  bool solve_pending_{false};

  std::deque<std::array<double, NX>> states_;
  std::deque<Measurement> measurements_;
  std::array<double, NX> arrival_state_{};
  std::array<double, NX> process_sigma_base_{};
  std::array<double, NX> arrival_sigma_{};
  MarginalPrior marginal_prior_;
  std::unique_ptr<ceres::Problem> graph_problem_;
  std::vector<GraphEntry> graph_entries_;
  ceres::ResidualBlockId graph_prior_id_{nullptr};
  size_t graph_dirty_from_{static_cast<size_t>(-1)};
  uint64_t graph_full_build_count_{0};
  uint64_t graph_updated_state_count_{0};
  uint64_t graph_prune_count_{0};
  uint64_t graph_reset_count_{0};
  uint64_t schur_numerical_fallback_count_{0};
  size_t last_marginal_candidate_count_{0};
  size_t last_marginal_evaluation_count_{0};
  bool last_marginal_full_scan_{true};

  double nis_wheel_left_{1.0};
  double nis_wheel_right_{1.0};
  double nis_gyro_{1.0};
  double nis_accel_{1.0};
  double raw_nis_wheel_left_{0.0};
  double raw_nis_wheel_right_{0.0};
  double raw_nis_gyro_{0.0};
  double raw_nis_accel_{0.0};
  bool nis_left_initialized_{false};
  bool nis_right_initialized_{false};
  bool nis_gyro_initialized_{false};
  bool nis_accel_initialized_{false};
  double current_r_scale_left_{1.0};
  double current_r_scale_right_{1.0};
  double current_r_scale_gyro_{1.0};
  double current_r_scale_accel_{1.0};
  double qv_scale_{1.0};
  double qw_scale_{1.0};
  double confidence_{1.0};

  AdaptiveGate gate_left_;
  AdaptiveGate gate_right_;
  AdaptiveGate gate_gyro_;
  AdaptiveGate gate_accel_;

  double common_slip_observability_{0.0};
  double differential_slip_observability_{0.0};
  double marginal_condition_{1.0};
  std::array<double, 4> gate_state_last_{{0.0, 0.0, 0.0, 0.0}};
  std::array<uint64_t, 4> gate_reject_counts_{{0, 0, 0, 0}};
  bool have_last_accepted_endpoint_{false};
  uint64_t accepted_solution_count_{0};
  double last_accepted_v_{0.0};
  double last_accepted_w_{0.0};
  rclcpp::Time last_accepted_stamp_{0, 0, RCL_ROS_TIME};
  double last_initial_cost_{0.0};
  double last_final_cost_{0.0};
  double last_iteration_count_{0.0};
  uint64_t solver_rollback_count_{0};
  uint64_t qr_fallback_attempt_count_{0};
  uint64_t qr_fallback_skip_count_{0};
  int last_rejection_reason_{0};
  uint64_t solver_deadline_misses_{0};
  uint64_t callback_deadline_misses_{0};
  uint64_t tick_late_count_{0};
  uint64_t covariance_deferred_count_{0};
  uint64_t covariance_forced_count_{0};
  double last_graph_build_ms_{0.0};
  double last_ceres_ms_{0.0};
  double last_covariance_ms_{0.0};
  double last_marginalization_ms_{0.0};
  double last_callback_wall_ms_{0.0};
  double last_wall_tick_period_ms_{0.0};
  double covariance_runtime_ewma_ms_{0.0};
  bool have_last_wall_tick_{false};
  std::chrono::steady_clock::time_point last_wall_tick_start_{};
  Eigen::Matrix3d pub_pose_covariance_{Eigen::Matrix3d::Identity() * 0.0004};
  double common_slip_memory_{0.0};
  bool common_slip_memory_valid_{false};
  rclcpp::Time common_slip_memory_last_stamp_{0, 0, RCL_ROS_TIME};

  MatX latest_covariance_{MatX::Identity()};
  bool covariance_valid_{false};
  size_t covariance_solve_counter_{0};
  size_t covariance_age_solves_{0};
  size_t covariance_failure_count_{0};
  size_t covariance_retry_after_solve_{0};
  fast_cov::Worker covariance_worker_;
  uint64_t covariance_epoch_{0};
  uint64_t covariance_latest_sequence_{0};
  bool covariance_job_in_flight_{false};
  uint64_t covariance_jobs_submitted_{0};
  uint64_t covariance_jobs_completed_{0};
  uint64_t covariance_discarded_{0};
  double covariance_worker_compute_ms_{0.0};
  double covariance_snapshot_ewma_ms_{0.0};

  // Pre-fit innovations are computed from the prior state before the incoming
  // measurement is used by warm-start or Ceres.
  double prefit_e_left_{0.0};
  double prefit_e_right_{0.0};
  double prefit_e_gyro_{0.0};
  double prefit_e_accel_{0.0};

  // Post-fit residuals are diagnostic only and never drive adaptive R.
  double last_e_left_{0.0};
  double last_e_right_{0.0};
  double last_e_gyro_{0.0};
  double last_e_accel_{0.0};

  bool have_previous_gyro_{false};
  double previous_gyro_corrected_{0.0};
  rclcpp::Time previous_gyro_stamp_{0, 0, RCL_ROS_TIME};

  int stationary_counter_{0};
  bool stationary_latched_{false};

  bool accel_filter_initialized_{false};
  double accel_filtered_{0.0};
  double accel_raw_longitudinal_{0.0};
  double accel_corrected_last_{0.0};
  bool accel_gravity_compensated_last_{false};
  bool accel_zero_initialized_{false};
  int accel_zero_counter_{0};
  double accel_zero_offset_{0.0};
  bool have_previous_corrected_accel_{false};
  double previous_corrected_accel_{0.0};
  rclcpp::Time previous_corrected_accel_stamp_{0, 0, RCL_ROS_TIME};
  bool latest_odom_quiet_{false};
  bool have_last_odom_for_accel_{false};
  double last_odom_v_for_accel_{0.0};
  rclcpp::Time last_odom_stamp_for_accel_{0, 0, RCL_ROS_TIME};
  double wheel_accel_excitation_{0.0};

  bool last_solution_usable_{true};
  double last_solver_time_ms_{0.0};

  bool have_last_ros_tick_{false};
  rclcpp::Time last_ros_tick_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_measurement_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_processed_sensor_stamp_{0, 0, RCL_ROS_TIME};
  bool have_last_imu_stamp_{false};
  rclcpp::Time last_imu_stamp_{0, 0, RCL_ROS_TIME};
  double last_sensor_age_sec_{0.0};
  double last_measured_left_{0.0};
  double last_measured_right_{0.0};

  // Continuous published odometry state. This is intentionally separate from
  // the sliding-window optimizer endpoint to prevent nonlinear re-solve jitter.
  bool pub_state_initialized_{false};
  double pub_x_{0.0};
  double pub_y_{0.0};
  double pub_yaw_{0.0};
  bool output_shaper_initialized_{false};
  double pub_v_{0.0};
  double pub_w_{0.0};
  double pub_linear_accel_{0.0};
  double pub_angular_accel_{0.0};

  std::string odom_topic_;
  std::string imu_topic_;
  std::string output_topic_;
  std::string odom_frame_;
  std::string base_frame_;

  bool publish_tf_{true};
  double wheel_separation_{0.30};
  double frequency_{100.0};
  int window_size_{12};
  int max_iterations_{4};
  double solver_budget_ms_{6.0};
  double transform_time_offset_{0.02};
  bool batch_solve_enabled_{true};
  int dense_qr_max_states_{16};
  bool prefer_normal_cholesky_{true};
  bool analytic_factors_enabled_{true};
  bool so2_yaw_residual_enabled_{false};
  bool gyro_increment_factor_enabled_{false};
  double gyro_increment_model_sigma_{0.12};
  double gyro_increment_max_dt_{0.030};
  bool exact_se2_motion_enabled_{false};
  bool incremental_graph_enabled_{false};
  bool block_schur_enabled_{false};
  bool rank_aware_prior_enabled_{false};
  double prior_eigen_relative_cutoff_{1e-9};
  double prior_max_discarded_gradient_fraction_{1e-4};
  int marginal_prior_rank_{NX};
  double marginal_prior_discarded_gradient_{0.0};
  uint64_t rank_prior_update_count_{0};
  uint64_t rank_prior_reject_count_{0};
  double qr_fallback_min_remaining_ms_{1.0};
  bool qr_fallback_on_endpoint_jump_{false};
  double rt_deadline_ms_{10.0};
  int last_solver_mode_{0};

  double horizon_duration_sec_{0.12};
  int max_horizon_states_{32};
  int max_events_per_tick_{16};
  int max_event_queue_size_{256};
  double merge_tolerance_sec_{0.0008};
  double out_of_order_tolerance_sec_{0.002};
  double reset_gap_sec_{0.25};
  double sensor_timeout_sec_{0.15};
  bool timestamp_aligned_output_enabled_{true};
  double max_propagation_gap_sec_{0.05};
  double future_tolerance_sec_{0.002};
  bool require_odom_for_init_{true};

  double output_linear_deadband_{0.002};
  double output_angular_deadband_{0.004};
  bool output_smoothing_enabled_{true};
  bool output_smoothing_linear_enabled_{true};
  bool output_smoothing_angular_enabled_{false};
  mhe_sensor_fusion::YawFeedbackPolicy yaw_feedback_policy_{};
  se2_correct::Policy se2_correction_policy_{};
  bool last_se2_correction_trusted_{false};
  double last_se2_correction_distance_{0.0};
  double last_se2_correction_yaw_{0.0};
  uint64_t se2_correction_applied_count_{0};
  double last_yaw_feedback_error_{0.0};
  double last_yaw_feedback_correction_{0.0};
  uint64_t yaw_feedback_applied_count_{0};
  double output_linear_tau_{0.030};
  double output_angular_tau_{0.018};
  bool output_median3_enabled_{true};
  bool output_adaptive_tau_enabled_{true};
  double output_linear_quiet_tau_{0.060};
  double output_linear_fast_tau_{0.020};
  double output_angular_quiet_tau_{0.040};
  double output_angular_fast_tau_{0.008};
  double output_linear_transition_{0.060};
  double output_angular_transition_{0.150};
  double output_max_linear_accel_{3.0};
  double output_max_linear_decel_{4.0};
  double output_max_angular_accel_{12.0};
  double output_max_angular_decel_{16.0};
  double output_max_linear_jerk_{60.0};
  double output_max_angular_jerk_{240.0};
  std::deque<double> output_v_target_history_;
  std::deque<double> output_w_target_history_;
  double output_guarded_v_target_{0.0};
  double output_guarded_w_target_{0.0};
  double output_effective_linear_tau_{0.030};
  double output_effective_angular_tau_{0.018};

  bool stationary_enabled_{true};
  double stationary_v_threshold_{0.012};
  double stationary_w_threshold_{0.025};
  double stationary_gyro_threshold_{0.025};
  double stationary_accel_threshold_{0.18};
  int stationary_min_samples_{4};
  double stationary_sigma_v_{0.004};
  double stationary_sigma_w_{0.008};

  double process_sigma_v_{0.10};
  double process_sigma_w_{0.22};

  double gyro_scale_{1.0};
  double accel_scale_{1.0};
  double accel_lpf_tau_{0.05};
  bool accel_gravity_compensation_{false};
  double gravity_mps2_{9.80665};
  std::vector<double> accel_axis_{1.0, 0.0, 0.0};
  bool accel_auto_zero_enabled_{true};
  double accel_auto_zero_alpha_{0.02};
  double accel_auto_zero_max_abs_offset_{12.0};
  double accel_max_jerk_{30.0};
  double accel_loss_delta_{1.5};

  double sigma_wheel_left_base_{0.025};
  double sigma_wheel_right_base_{0.025};
  double sigma_gyro_base_{0.025};
  double sigma_accel_base_{0.35};

  bool adaptive_enabled_{true};
  double adaptive_alpha_{0.05};
  double max_wheel_r_scale_{3.0};
  double max_sensor_r_scale_{3.0};
  double max_q_scale_{5.0};
  double qv_gain_{1.5};
  double qw_gain_{1.5};
  double accel_reference_{0.50};
  double angular_accel_reference_{1.50};
  double adaptive_q_scale_alpha_{0.08};
  double adaptive_nis_high_{4.0};
  double adaptive_nis_low_{2.0};
  int adaptive_enter_samples_{5};
  int adaptive_exit_samples_{20};
  double adaptive_scale_step_up_{0.20};
  double adaptive_scale_step_down_{0.04};
  double prefit_wheel_model_sigma_{0.040};
  double prefit_gyro_model_sigma_{0.025};
  double prefit_accel_model_sigma_{0.50};
  bool innovation_gating_enabled_{true};
  double gate_wheel_soft_nis_{9.0};
  double gate_wheel_hard_nis_{64.0};
  double gate_gyro_soft_nis_{9.0};
  double gate_gyro_hard_nis_{64.0};
  double gate_accel_soft_nis_{16.0};
  double gate_accel_hard_nis_{100.0};
  double gate_max_sigma_scale_{3.0};
  int gate_warmup_accepted_solves_{5};
  double gate_large_gap_sec_{0.035};
  double solver_cost_tolerance_{1e-4};
  double solver_max_endpoint_linear_accel_{8.0};
  double solver_max_endpoint_angular_accel_{30.0};
  double solver_linear_jump_margin_{0.35};
  double solver_angular_jump_margin_{1.0};
  double common_memory_max_prefit_nis_{9.0};

  double slip_left_min_{-0.40};
  double slip_left_max_{1.50};
  double slip_right_min_{-0.40};
  double slip_right_max_{1.50};
  double slip_accel_observable_{0.40};
  double slip_angular_observable_{0.35};
  double sigma_slip_common_unobservable_{0.035};
  double sigma_slip_common_active_{0.15};
  double sigma_slip_diff_unobservable_{0.040};
  double sigma_slip_diff_active_{0.30};
  double sigma_sl_process_{0.025};
  double sigma_sr_process_{0.025};
  double slip_warm_start_decay_{0.998};
  double sigma_slip_common_hold_{0.080};
  double common_slip_memory_tau_{8.0};
  double common_slip_learn_observability_{0.35};
  double slip_min_speed_observable_{0.05};
  double sigma_common_slip_accel_{0.35};
  double common_slip_accel_pair_max_age_{0.030};
  double common_slip_memory_alpha_min_{0.05};
  double common_slip_memory_alpha_max_{0.20};

  bool covariance_enabled_{true};
  int covariance_update_every_n_{20};
  int covariance_max_age_solves_{60};
  double covariance_rt_budget_ms_{8.0};
  double covariance_rt_reserve_ms_{2.5};
  bool covariance_defer_if_slow_{true};
  int covariance_max_states_{20};
  double covariance_min_variance_{1e-9};
  double covariance_max_variance_{100.0};
  bool covariance_async_enabled_{true};
  double covariance_snapshot_budget_ms_{4.5};
  int covariance_max_stale_solves_{120};
  bool covariance_jacobi_scaled_enabled_{false};
  bool covariance_svd_fallback_enabled_{true};
  double covariance_minimum_rcond_{1e-10};
  double covariance_svd_relative_cutoff_{1e-11};
  double covariance_maximum_svd_condition_{1e10};
  int covariance_last_method_{0};
  int covariance_last_rank_{0};
  double covariance_last_rcond_{0.0};
  // Disabled by default; per-axis std dev factors are only for validated,
  // independent-ground-truth calibration (never auto-fit online).
  bool pose_covariance_calibration_enabled_{false};
  Eigen::Vector3d pose_covariance_std_scales_{Eigen::Vector3d::Ones()};

  double max_abs_v_{1.5};
  double max_abs_w_{6.0};
  double max_abs_gyro_{8.0};
  double max_abs_accel_{15.0};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr confidence_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr innovation_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr bias_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr solve_time_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr marginal_condition_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_slip_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr sensor_health_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr raw_nis_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr postfit_residual_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr timing_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr time_alignment_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr covariance_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr accel_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr output_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gating_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr solver_health_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr realtime_profile_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr graph_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr covariance_worker_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mhe_fusion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mhe_fusion::MheFusionNode>());
  rclcpp::shutdown();
  return 0;
}
