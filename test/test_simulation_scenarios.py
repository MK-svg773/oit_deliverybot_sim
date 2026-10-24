"""Check shared seeded schedules and the separate slow following range."""
from pathlib import Path
import random
import sys

import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from simulation_scenarios import position_at, resolve_scenario, speed_at  # noqa: E402, I100


def test_shared_seed_and_piecewise_stops():
    """Keep seeded schedules identical across methods and integrate each segment."""
    config = yaml.safe_load(
        (Path(__file__).resolve().parents[1] /
         'config/simulation/scenarios.yaml').read_text())
    a = resolve_scenario(config, 'stop_go', 1)
    assert a == resolve_scenario(config, 'stop_go', 1)
    assert a != resolve_scenario(config, 'stop_go', 2)
    delay = a['delay']
    assert speed_at(a, delay - 1) == 0
    assert speed_at(a, delay + .5) > 0
    assert speed_at(a, delay + 1) == 0
    assert speed_at(a, delay + 3) > 0
    assert position_at(a, delay + .8) == position_at(a, delay + 2.6)
    b = resolve_scenario(config, 'oncoming', 1)
    assert len(b['segments']) == 1
    assert abs(position_at(b, 100)[0] - (7 - b['segments'][0]['speed'] * 6.5)) < 1e-9


def test_following_uses_its_own_speed_range():
    """Prevent pedestrian variation from slowing following below its safe endpoint."""
    config = {'_variation': {'speed_scale': [0.6, 1.2], 'delay_offset_s': [0, 0]},
              'following': {'x': 1.8, 'y': 0, 'yaw': 0, 'delay': 0,
                            'speed': 0.25, 'duration': 24, 'speed_scale': [0.9, 1.0]}}
    for seed in range(1, 6):
        obstacle = resolve_scenario(config, 'following', seed)
        assert 0.225 <= speed_at(obstacle, 1) <= 0.25
        assert position_at(obstacle, 100)[0] >= 7.2


def test_oncoming_lateral_offset_is_seeded_without_changing_timing():
    """Catch missing offsets and accidental changes to the existing random draws."""
    config = yaml.safe_load(
        (Path(__file__).resolve().parents[1] /
         'config/simulation/scenarios.yaml').read_text())
    signs = set()
    for seed in range(1, 31):
        obstacle = resolve_scenario(config, 'oncoming', seed)
        assert obstacle == resolve_scenario(config, 'oncoming', seed)
        assert 0.3 <= abs(obstacle['y']) <= 0.7
        signs.add(obstacle['y'] > 0)
        rng = random.Random(seed)
        assert obstacle['delay'] == rng.uniform(0, 0.3)
        assert obstacle['speed_scale'] == rng.uniform(0.6, 1.2)
        assert obstacle['x'] == 7
        assert obstacle['segments'][0]['duration'] == 6.5
        assert abs(position_at(obstacle, 100)[1] - obstacle['y']) < 1e-9
    assert signs == {False, True}
