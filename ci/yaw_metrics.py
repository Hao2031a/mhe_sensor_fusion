"""ROS-free time-aligned yaw metrics used for Gazebo ground-truth A/B tests."""
from bisect import bisect_right
from math import atan2, cos, sin, sqrt


def wrap(angle):
    return atan2(sin(angle), cos(angle))


def interpolate_yaw(samples, t, max_gap=0.10):
    """Use ROS header times, never subscriber arrival times."""
    if not samples or t < samples[0][0] or t > samples[-1][0]:
        return None
    times = [row[0] for row in samples]
    j = bisect_right(times, t)
    if j == 0:
        return None
    if j >= len(samples):
        if abs(t - samples[-1][0]) <= 1e-8:
            return samples[-1][1]
        return None
    t0, y0 = samples[j - 1]
    t1, y1 = samples[j]
    dt = t1 - t0
    if dt < 1e-9 or dt > max_gap:
        return None
    return wrap(y0 + wrap(y1 - y0) * (t - t0) / dt)


def evaluate(truth, estimate, max_gap=0.10):
    truth = sorted(truth)
    estimate = sorted(estimate)
    pairs = []
    for t, yaw in estimate:
        ref = interpolate_yaw(truth, t, max_gap)
        if ref is not None:
            pairs.append((t, yaw, ref))
    if len(pairs) < 10:
        raise ValueError('Insufficient synchronized samples, or wrong truth topic/time source')
    # Offset-independent relative yaw drift; comparison does not mistake a
    # constant orientation difference between two odom frames for drift.
    offset = wrap(pairs[0][1] - pairs[0][2])
    errors = [wrap(y - ref - offset) for _, y, ref in pairs]
    absolute = sorted(abs(e) for e in errors)
    return dict(
        n=len(errors),
        span_sec=pairs[-1][0] - pairs[0][0],
        relative_yaw_rmse_rad=sqrt(sum(e * e for e in errors) / len(errors)),
        relative_yaw_p95_rad=absolute[int(0.95 * (len(absolute) - 1))],
        relative_yaw_max_rad=absolute[-1],
        relative_yaw_final_rad=errors[-1],
    )
