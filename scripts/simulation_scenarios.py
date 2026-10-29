"""Seeded schedules shared by the simulator, monitor and numerical checks."""
import copy
import math
import random


# Keep scenario:=all equal to the original final-evaluation set.
BASE_SCENARIOS = ('clear', 'static', 'crossing', 'following', 'oncoming',
                  'stop_go', 'speed_change')


def resolve_scenario(config, name, seed, origin=None):
    """Resolve one seeded obstacle schedule without mutating configuration."""
    if origin is not None and (len(origin) != 3 or not all(math.isfinite(v) for v in origin)):
        raise ValueError('Scenario origin must be finite [x, y, yaw]')
    obstacle = copy.deepcopy(config[name])
    if obstacle is None:
        return None
    variation = config['_variation']
    rng = random.Random(seed)
    obstacle['delay'] += rng.uniform(*variation['delay_offset_s'])
    scale = rng.uniform(*obstacle.get('speed_scale', variation['speed_scale']))
    obstacle['speed_scale'] = scale
    segments = obstacle.get('segments', [{'speed': obstacle.get('speed', 0.),
                                          'duration': obstacle.get('duration', 0.)}])
    obstacle['segments'] = [{'speed': s['speed'] * scale, 'duration': s['duration']}
                            for s in segments]
    if name == 'oncoming':
        obstacle['y'] = rng.uniform(0.3, 0.7) * rng.choice((-1, 1))
    if origin is not None:
        ox, oy, yaw = origin
        x, y = obstacle['x'], obstacle['y']
        obstacle['x'] = ox + math.cos(yaw) * x - math.sin(yaw) * y
        obstacle['y'] = oy + math.sin(yaw) * x + math.cos(yaw) * y
        obstacle['yaw'] += yaw
    return obstacle


def speed_at(obstacle, elapsed):
    """Return commanded longitudinal speed in a half-open schedule interval."""
    t = elapsed - obstacle['delay']
    if t < 0:
        return 0.
    for segment in obstacle['segments']:
        if t < segment['duration']:
            return segment['speed']
        t -= segment['duration']
    return 0.


def position_at(obstacle, elapsed):
    """Integrate the schedule to its expected world position."""
    t = max(0., elapsed - obstacle['delay'])
    distance = 0.
    for segment in obstacle['segments']:
        distance += min(t, segment['duration']) * segment['speed']
        t = max(0., t - segment['duration'])
    return (obstacle['x'] + math.cos(obstacle['yaw']) * distance,
            obstacle['y'] + math.sin(obstacle['yaw']) * distance)
