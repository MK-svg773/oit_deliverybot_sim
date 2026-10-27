"""Small, ROS-independent calculations for the navigation experiments."""
import math


def support_contact(names, relative_heights, normal_z):
    """Support on a near-level floor; don't discard an entire school mesh."""
    return (any(part in names for part in ('left_wheel', 'right_wheel', 'caster')) and
            bool(relative_heights) and max(relative_heights) <= 0.035 and
            bool(normal_z) and all(abs(z) > 0.8 for z in normal_z))


def interpolate_truth(history, stamp, max_gap=0.12):
    """Interpolate two bracketing truth samples; never treat stale data as truth."""
    for left, right in zip(history, history[1:]):
        if left[0] <= stamp <= right[0] and 0 < right[0] - left[0] <= max_gap:
            f = (stamp - left[0]) / (right[0] - left[0])
            return left[1] + f * (right[1] - left[1]), left[2] + f * (right[2] - left[2])
    raise ValueError('No fresh bracketing obstacle truth samples')


def clearance_to_circle(x, y, yaw, cx, cy, radius):
    """Planar gap from deliverybot's configured rectangle to a circular obstacle."""
    dx, dy = cx - x, cy - y
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    return max(0.0, math.hypot(max(-0.35 - local_x, 0.0, local_x - 0.11),
                               max(abs(local_y) - 0.22, 0.0)) - radius)


class TrialMetrics:
    def __init__(self, stop_speed=0.03, start_time=None):
        self.stop_speed = stop_speed
        self.first, self.last = start_time, None
        self.distance = self.stopped = self.speed_variation = 0.0
        self.minimum = None

    def add(self, t, x, y, speed, clearance):
        if not all(math.isfinite(v) for v in (t, x, y, speed)):
            raise ValueError('Nonfinite sample')
        if self.last:
            dt = t - self.last[0]
            if dt < 0:
                raise ValueError('Simulation time moved backwards within a trial')
            self.distance += math.hypot(x - self.last[1], y - self.last[2])
            if abs(self.last[3]) < self.stop_speed:
                self.stopped += dt
            self.speed_variation += abs(speed - self.last[3])
        if clearance is not None and math.isfinite(clearance):
            self.minimum = clearance if self.minimum is None else min(self.minimum, clearance)
        self.first = t if self.first is None else self.first
        self.last = (t, x, y, speed)

    def result(self, outcome, terminal_time=None):
        end = terminal_time if terminal_time is not None else self.last[0] if self.last else None
        duration = max(0., end - self.first) if end is not None and self.first is not None else 0.0
        return {
            'duration_s': duration,
            'arrival_s': duration if outcome == 'success' else None,
            'distance_m': self.distance,
            'stopped_s': self.stopped,
            'min_clearance_m': self.minimum,
            'speed_variation_mps': self.speed_variation,
        }


def post_clearance_metrics(rows, environment, obstacle, terminal_time):
    """
    Score crossing exit from sampled truth only, without extrapolating beyond a trial.

    The lane is the straight start-goal centreline swept by the 0.44 m wide body,
    enlarged by a fixed margin. Exit is the first fresh sample fully beyond the
    exit side, not an interpolated or scheduled timestamp. Sample speeds
    represent their following interval, as in TrialMetrics. Gaps over 0.15 s are
    missing; restart requires a continuously observed hold inside the post window.
    """
    settings = obstacle.get('post_clearance') if obstacle else None
    result = {
        't_clear_status': 'not_applicable', 't_clear_s': None, 'w_post_s': None,
        'post_observed_s': None, 'post_missing_s': None, 'post_stopped_s': None,
        'post_window_complete': False, 'restart_category': 'not_applicable',
        'restart_delay_s': None, 'restart_observed_s': None,
    }
    if not settings:
        return result
    window, hold = settings['window_s'], settings['hold_s']
    result.update(t_clear_status='not_observed', w_post_s=window,
                  restart_category='clear_not_observed')
    sx, sy = environment['start'][:2]
    gx, gy = environment['goal'][:2]
    length = math.hypot(gx - sx, gy - sy)

    def lateral(x, y):
        return ((gx - sx) * (y - sy) - (gy - sy) * (x - sx)) / length

    # The initial circle can already be inside the lane; heading defines the exit side.
    across = (gx - sx) * math.sin(obstacle['yaw']) - (gy - sy) * math.cos(obstacle['yaw'])
    direction = 1. if across > 0 else -1.
    edge = .22 + settings['margin_m'] + environment['obstacle_radius']
    samples = [(float(row['time_s']), float(row['speed_mps']),
                direction * lateral(float(row['obstacle_x_m']), float(row['obstacle_y_m'])))
               for row in rows if float(row['time_s']) <= terminal_time]
    max_gap = .15
    clear_index = None
    for i, sample in enumerate(samples):
        if sample[2] <= edge:
            continue
        if (i == 0 or not 0 < sample[0] - samples[i - 1][0] <= max_gap or
                not all(math.isfinite(v) for v in (*sample, *samples[i - 1]))):
            result['t_clear_status'] = 'missing_samples'
            return result
        clear_index = i
        break
    if clear_index is None:
        return result
    if any(sample[2] <= edge for sample in samples[clear_index + 1:]):
        result['t_clear_status'] = 'reentered'
        return result
    t_clear, speed, _ = samples[clear_index]
    end = min(t_clear + window, terminal_time, samples[-1][0])
    observed = stopped = 0.
    moving_since = None
    restart = None
    missing_before_restart = False
    for left, right in zip(samples[clear_index:], samples[clear_index + 1:]):
        a, b = max(t_clear, left[0]), min(end, right[0])
        if b <= a:
            continue
        if (right[0] - left[0] > max_gap or
                not all(math.isfinite(v) for v in (*left, *right))):
            moving_since = None
            if restart is None:
                missing_before_restart = True
            continue
        observed += b - a
        if left[1] < environment['stop_speed']:
            stopped += b - a
            moving_since = None
        else:
            moving_since = a if moving_since is None else moving_since
            if restart is None and b - moving_since >= hold - 1e-9:
                restart = moving_since - t_clear
    complete = observed >= window - 1e-9
    if speed >= environment['stop_speed']:
        category = 'already_moving'
        restart = None
    elif missing_before_restart:
        category = 'missing_samples'
        restart = None
    elif restart is not None:
        category = 'restarted'
    else:
        category = 'censored_window' if complete else 'censored_end'
    result.update(t_clear_status='observed', t_clear_s=t_clear,
                  post_observed_s=observed, post_missing_s=max(0., end - t_clear - observed),
                  post_stopped_s=stopped, post_window_complete=complete,
                  restart_category=category, restart_delay_s=restart,
                  restart_observed_s=end - t_clear)
    return result
