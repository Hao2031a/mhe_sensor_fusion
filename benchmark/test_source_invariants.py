"""Source-level integration checks for guards; ROS runtime tests are still required."""
from pathlib import Path
s = (Path(__file__).resolve().parent.parent / 'src/mhe_sensor_fusion.cpp').read_text()
checks = {
    'incremental graph reuses problem and prunes prefix': (
        'prepareGraph()' in s and 'advanceGraphAfterMarginalization(' in s
        and 'enable_fast_removal = true' in s and 'graph_dirty_from_' in s),
    'block Schur chain selected and retains dense reference': (
        'block_schur::Chain<NX>' in s and 'chain.eliminate(' in s
        and 'if (block_schur_enabled_)' in s),
    'analytic Ceres factors A/B switch': ('analytic_factors_enabled_' in s and 'new AnalyticProcessCost' in s and 'new AnalyticWheelPairCost' in s),
    'independent angular bypass': (
        'output_smoothing_angular_enabled_' in s and
        'output_guarded_w_target_ = desired_w;' in s and
        'pub_w_ = desired_w;' in s),
    'yaw feedback outlier/timestamp protection': (
        'boundedYawFeedback(' in s and 'trustworthyYawEndpoint(' in s and
        'last_solution_usable_' in s and 'reference_skew' in s and
        'yaw_feedback_policy_.enabled && !force_hold && !sensor_stale' in s),
    'prefit gating before warm start': s.index('applyInnovationGating(z)') < s.index('updateWarmStartFromMeasurement(x, z)'),
    'reject out-of-order before appending state': s.index('(delta < 0.0 && -delta > merge_tolerance_sec_)') < s.index('states_.push_back(x);', s.index('void processSensorEvent')),
    'rollback protects slip memory': 'if (last_solution_usable_ && z.has_common_slip_accel_pair' in s,
    'Ceres cost evaluated': 'safety::costAcceptable(' in s,
    'Ceres endpoint jump evaluated': 'safety::endpointJumpAcceptable(' in s,
    'QR fallback uses remaining time budget': (
        'safety::planQrFallback(' in s and
        'options.max_solver_time_in_seconds = qr_plan.remaining_ms / 1000.0;' in s),
    'Ceres fallback can be skipped on endpoint rejection': (
        'qr_fallback_on_endpoint_jump_' in s and
        'last_rejection_reason_' in s),
    'output covariance propagated': 'pub_pose_covariance_ = F * pub_pose_covariance_ * F.transpose()' in s,
    'output covariance used for published pose': (
        'published_pose_covariance = pub_pose_covariance_ * pose_inflation' in s and
        'msg.pose.covariance[0] = published_pose_covariance(0, 0)' in s and
        'msg.pose.covariance[35] = published_pose_covariance(2, 2)' in s),
    'opt-in covariance calibration guarded': (
        'if (pose_covariance_calibration_enabled_)' in s and
        'calibratePublishedPoseCovariance(' in s),
    'R snapshot retains gate inflation': 'incoming.qv_scale = qv_scale_' in s,
    'future-dated sensor samples held in queue': (
        'eventIsDue(' in s and 'sensor_event_queue_.erase(it)' in s),
    'forward ROS clock gap never silently clamped': (
        'time_align::planStep(' in s and 'aligned_step.dt_sec' in s and
        'publishState(ros_now, publish_dt, hold_output)' in s),
}
for name, ok in checks.items():
    print(f'{"PASS" if ok else "FAIL"} {name}')
if not all(checks.values()):
    raise SystemExit('Source invariants FAIL')

# Additional integration invariants for asynchronous covariance scheduling.
assert 'acceptFastCovariance();' in s, 'worker completion not polled on publish tick'
assert 'covariance_worker_.submit(std::move(snapshot))' in s, 'async worker submit missing'
assert 'covariance_job_in_flight_' in s, 'in-flight coalescing missing'
assert 'result.generation != covariance_epoch_' in s, 'reset generation protection missing'
assert 'ceres::DENSE_SVD' not in s, 'blocking SVD still present in callback path'
print('PASS asynchronous covariance snapshot, mailbox, generation and no sync SVD')
