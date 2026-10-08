from pathlib import Path
p=Path(__file__).parent/'src/mhe_sensor_fusion.cpp'
s=p.read_text()
def sub(old,new,label):
 global s
 assert old in s, label
 s=s.replace(old,new,1)
sub('#include "mhe_sensor_fusion/low_latency_output.hpp"','#include "mhe_sensor_fusion/low_latency_output.hpp"\n#include "mhe_sensor_fusion/time_aligned_output.hpp"','include')
sub('    sensor_timeout_sec_ = declare_parameter<double>("timing.sensor_timeout", 0.15);','''    sensor_timeout_sec_ = declare_parameter<double>("timing.sensor_timeout", 0.15);
    timestamp_aligned_output_enabled_ = declare_parameter<bool>(
      "timing.timestamp_aligned_output_enabled", true);
    max_propagation_gap_sec_ = declare_parameter<double>(
      "timing.max_propagation_gap", 0.05);
    future_tolerance_sec_ = declare_parameter<double>(
      "timing.future_tolerance", 0.002);''','timing params')
sub('    yaw_feedback_policy_.max_sensor_age_sec = declare_parameter<double>(\n      "output.yaw_feedback.max_sensor_age", 0.04);','''    yaw_feedback_policy_.max_sensor_age_sec = declare_parameter<double>(
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
    yaw_feedback_policy_.max_covariance_age_solves = static_cast<unsigned int>(std::max(0,
      declare_parameter<int>("output.yaw_feedback.max_covariance_age_solves", 80)));''','yaw params')
sub('    timing_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/mhe/timing", 10);','''    timing_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/mhe/timing", 10);
    time_alignment_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/mhe/time_alignment", 10);''','publisher init')
sub('  std::vector<SensorEvent> drainSensorEvents()','  std::vector<SensorEvent> drainSensorEvents(const rclcpp::Time & ros_now)','drain signature')
sub('''      const int count = std::min<int>(
        static_cast<int>(sensor_event_queue_.size()),
        std::max(1, max_events_per_tick_));

      events.reserve(static_cast<size_t>(count));
      for (int i = 0; i < count; ++i) {
        events.push_back(std::move(sensor_event_queue_.front()));
        sensor_event_queue_.pop_front();
      }
      last_queue_depth_ = sensor_event_queue_.size();''','''      const size_t max_count = static_cast<size_t>(std::max(1, max_events_per_tick_));
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
      last_queue_depth_ = sensor_event_queue_.size();''','drain body')
sub('''  void resetEstimatorForTimeJump()
  {
    resetHorizon(true);
    have_last_ros_tick_ = false;''','''  void resetEstimatorForTimeJump()
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
    have_last_ros_tick_ = false;''','reset jump')
# timer: first pass get raw dt (without clamp), then after events plan
sub('''    double publish_dt = 1.0 / frequency_;

    if (have_last_ros_tick_) {
      const double raw_dt = (ros_now - last_ros_tick_).seconds();
      if (raw_dt < -1e-6) {
        resetEstimatorForTimeJump();
        last_ros_tick_ = ros_now;
        have_last_ros_tick_ = true;
        return;
      }
      if (raw_dt <= 1e-6) {
        return;
      }
      publish_dt = std::clamp(raw_dt, 0.001, 0.050);
    }
    last_ros_tick_ = ros_now;
    have_last_ros_tick_ = true;

    acceptFastCovariance();
    auto events = drainSensorEvents();''','''    const bool had_prior_tick = have_last_ros_tick_;
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
    auto events = drainSensorEvents(ros_now);''','timer dt')
sub('''    if (!states_.empty()) {
      publishState(ros_now, publish_dt);
    }
    last_callback_wall_ms_''','''    // Determine the integration step AFTER consuming sensor events; otherwise
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
    last_callback_wall_ms_''','timer publish')
sub('  void publishState(const rclcpp::Time & stamp, double dt)','  void publishState(const rclcpp::Time & stamp, double dt, bool force_hold)','publish signature')
sub('    if (sensor_stale) {\n      desired_v = 0.0;','    if (sensor_stale || force_hold) {\n      desired_v = 0.0;','stale force')
sub('    if (sensor_stale || stationary_now) {','    if (sensor_stale || stationary_now || force_hold) {','force shaper')
sub('''    if (yaw_feedback_policy_.enabled && !sensor_stale && !stationary_now &&
        accepted_solution_count_ > 0 && last_solution_usable_ &&
        last_processed_sensor_stamp_.nanoseconds() > 0) {
      const double sensor_age = (stamp - last_processed_sensor_stamp_).seconds();
      const auto correction = mhe_sensor_fusion::boundedYawFeedback(
        pub_yaw_, x[YAW], x[W], sensor_age, dt, yaw_feedback_policy_);''','''    last_yaw_feedback_trusted_ = false;
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
        mhe_sensor_fusion::YawFeedbackResult{};''','trust feedback')
sub('''      static_cast<double>(yaw_feedback_applied_count_),
      yaw_feedback_policy_.enabled ? 1.0 : 0.0
    };''','''      static_cast<double>(yaw_feedback_applied_count_),
      yaw_feedback_policy_.enabled ? 1.0 : 0.0,
      // Appended to preserve legacy positions 0..23.
      timestamp_aligned_output_enabled_ ? 1.0 : 0.0,
      last_yaw_feedback_trusted_ ? 1.0 : 0.0,
      static_cast<double>(last_time_alignment_status_),
      last_yaw_reference_age_sec_
    };''','output diagnostics')
sub('  size_t last_queue_depth_{0};','''  size_t last_queue_depth_{0};
  size_t last_deferred_future_events_{0};
  uint64_t forward_clock_gap_count_{0};
  uint64_t backward_clock_jump_count_{0};
  double last_sim_tick_dt_sec_{0.0};
  int last_time_alignment_status_{1};
  bool last_yaw_feedback_trusted_{false};
  double last_yaw_reference_age_sec_{0.0};
  uint64_t yaw_feedback_trust_rejected_count_{0};''','members counters')
sub('  double sensor_timeout_sec_{0.15};','''  double sensor_timeout_sec_{0.15};
  bool timestamp_aligned_output_enabled_{true};
  double max_propagation_gap_sec_{0.05};
  double future_tolerance_sec_{0.002};''','member params')
sub('  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr timing_pub_;','''  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr timing_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr time_alignment_pub_;''','member publisher')
p.write_text(s)
