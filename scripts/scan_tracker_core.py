"""
Scan clustering and nearest-neighbour constant-velocity Kalman tracking.

No simulator state or obstacle schedule is accepted by this module.
"""
import math

import numpy as np
from scipy.ndimage import distance_transform_edt


def rotation(q):
    """Return planar yaw from a quaternion."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class StaticMap:
    """Cache distances to static occupied map cells."""

    def __init__(self, message, map_exclusion_distance=.20):
        """Precompute cell distances and retain map geometry."""
        self.map_exclusion_distance = map_exclusion_distance
        self.info = message.info
        self.yaw = rotation(self.info.origin.orientation)
        cells = np.asarray(message.data).reshape(self.info.height, self.info.width)
        self.distance = distance_transform_edt(cells < 50) * self.info.resolution

    def near_wall(self, point):
        """Reject points near static occupancy or outside the map."""
        dx = point[0] - self.info.origin.position.x
        dy = point[1] - self.info.origin.position.y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        x = int(math.floor((c * dx + s * dy) / self.info.resolution))
        y = int(math.floor((-s * dx + c * dy) / self.info.resolution))
        return (not (0 <= x < self.info.width and 0 <= y < self.info.height) or
                self.distance[y, x] <= self.map_exclusion_distance)


def detections(scan, transform, static_map):
    """Split adjacent beams after map filtering; fit visible circular arcs."""
    tx, ty, angle = transform
    origin = np.array([tx, ty])
    clusters, current = [], []
    previous = None
    previous_index = -10
    for index, distance in enumerate(scan.ranges):
        point = None
        if math.isfinite(distance) and scan.range_min <= distance <= scan.range_max:
            a = angle + scan.angle_min + index * scan.angle_increment
            candidate = origin + distance * np.array([math.cos(a), math.sin(a)])
            if not static_map.near_wall(candidate):
                point = candidate
        if point is None:
            continue
        # One missing beam can arise from projection/binning of the 3D scan.
        if previous is not None and (index - previous_index >
                                     2 or np.linalg.norm(point - previous) > .25):
            if len(current) >= 3:
                clusters.append(np.asarray(current))
            current = []
        current.append(point)
        previous, previous_index = point, index
    if len(current) >= 3:
        clusters.append(np.asarray(current))
    result = []
    for points in clusters:
        mean = points.mean(axis=0)
        if np.max(np.linalg.norm(points - mean, axis=1)) > .75:
            continue
        # Centre the least-squares system for numerical stability in large maps.
        local = points - mean
        eigenvalues = np.linalg.eigvalsh(local.T @ local / len(local))
        # Thin, straight fragments are map edges, not a compact obstacle arc.
        if eigenvalues[0] < .005 * max(eigenvalues[1], 1.e-9):
            continue
        a = np.column_stack([2 * local, np.ones(len(points))])
        fit, _, rank, _ = np.linalg.lstsq(a, np.sum(local**2, axis=1), rcond=None)
        radius = math.sqrt(max(0., fit[2] + fit[0]**2 + fit[1]**2))
        if rank == 3 and .08 <= radius <= .60:
            centre = mean + fit[:2]
        else:
            radius = max(.10, min(.60, np.max(np.linalg.norm(local, axis=1))))
            direction = mean - origin
            centre = mean + direction / max(np.linalg.norm(direction), 1.e-6) * radius * .5
        result.append((centre[0], centre[1], radius))
    return result


class Track:
    """Maintain a single planar constant-velocity Kalman state."""

    def __init__(self, stamp, detection):
        """Initialize position, velocity uncertainty and observed radius."""
        self.state = np.array([detection[0], detection[1], 0., 0.])
        self.covariance = np.diag([.03**2, .03**2, 1., 1.])
        self.stamp = self.seen = stamp
        self.radius = detection[2]
        self.hits = 1

    def predict(self, stamp):
        """Advance the state and covariance to the scan stamp."""
        dt = stamp - self.stamp
        f = np.eye(4)
        f[0, 2] = f[1, 3] = dt
        g = np.array([[dt**2 / 2, 0], [0, dt**2 / 2], [dt, 0], [0, dt]])
        self.state = f @ self.state
        self.covariance = f @ self.covariance @ f.T + .8**2 * g @ g.T
        self.stamp = stamp

    def update(self, detection):
        """Correct position and radius with an associated detection."""
        h = np.eye(2, 4)
        r = np.eye(2) * .03**2
        gain = np.linalg.solve(h @ self.covariance @ h.T + r,
                               h @ self.covariance).T
        self.state += gain @ (np.asarray(detection[:2]) - h @ self.state)
        residual = np.eye(4) - gain @ h
        self.covariance = residual @ self.covariance @ residual.T + gain @ r @ gain.T
        self.radius = .7 * self.radius + .3 * detection[2]
        self.seen = self.stamp
        self.hits += 1


class Tracker:
    """Associate scan detections with recent confirmed tracks."""

    def __init__(self):
        """Initialize an empty observation history."""
        self.tracks = []
        self.last_stamp = -math.inf

    def update(self, stamp, observations):
        """Predict, associate and return tracks at the observation stamp."""
        if stamp <= self.last_stamp:
            return []
        self.last_stamp = stamp
        self.tracks = [track for track in self.tracks if stamp - track.seen <= .6]
        for track in self.tracks:
            track.predict(stamp)
        pairs = sorted((np.linalg.norm(track.state[:2] - np.asarray(obs[:2])), i, j)
                       for i, track in enumerate(self.tracks)
                       for j, obs in enumerate(observations))
        used_tracks, used_obs = set(), set()
        for distance, i, j in pairs:
            if distance > .75 or i in used_tracks or j in used_obs:
                continue
            self.tracks[i].update(observations[j])
            used_tracks.add(i)
            used_obs.add(j)
        for j, obs in enumerate(observations):
            if j not in used_obs:
                self.tracks.append(Track(stamp, obs))
        return [(*track.state, track.radius) for track in self.tracks
                if track.hits >= 3 and stamp - track.seen <= .2]
