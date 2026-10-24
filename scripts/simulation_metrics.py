"""Small, ROS-independent calculations for the navigation experiments."""
import math


def support_contact(names, relative_heights, normal_z):
    """Support on a near-level floor; don't discard an entire school mesh."""
    return (any(part in names for part in ('left_wheel', 'right_wheel', 'caster'))
            and bool(relative_heights) and max(relative_heights) <= 0.035
            and bool(normal_z) and all(abs(z) > 0.8 for z in normal_z))


def interpolate_truth(history, stamp, max_gap=0.12):
    """Interpolate two bracketing truth samples; never treat stale data as truth."""
    for left, right in zip(history, history[1:]):
        if left[0] <= stamp <= right[0] and 0 < right[0]-left[0] <= max_gap:
            f=(stamp-left[0])/(right[0]-left[0])
            return left[1]+f*(right[1]-left[1]), left[2]+f*(right[2]-left[2])
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
        duration = max(0., end-self.first) if end is not None and self.first is not None else 0.0
        return dict(duration_s=duration, arrival_s=duration if outcome == 'success' else None,
                    distance_m=self.distance, stopped_s=self.stopped,
                    min_clearance_m=self.minimum, speed_variation_mps=self.speed_variation)
