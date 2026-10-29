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


def test_original_seven_schedules_are_bit_identical_to_e11b711():
    """Catch any changed config, random draws or motion for development seeds 1–999."""
    import hashlib
    import json

    expected = {
        'clear': '9e27319419509b0b3180dbc506d11b9e92b082d09a3021d2896fea8fa57de1be',
        'static': '6aa17c6c00f0cc40706234c52089d47621cbcf883e65d061b5fd6e28a52d8a68',
        'crossing': '8b8478904971c7d4b4b557a21600132e0d6a9121925fd80fd3ef04386bc3cf5d',
        'following': '1ebc7abb16c48afb9a84d7be5ea9607ee064000dd7e54331af4766f35632c661',
        'oncoming': '7fc3cfffafab1dd89d0bc0ca234613bc2ce872253fa9be46629d93b7bf679aa7',
        'stop_go': 'd38ba69f7c34f400db1b4e68096eee7f35850f4379a46b21e722d3adc4903a91',
        'speed_change': '520fe9b587486633e9b99c125a37572acba6f817c7018ed3733cd2e3323520c7',
    }
    config = yaml.safe_load(
        (Path(__file__).resolve().parents[1] /
         'config/simulation/scenarios.yaml').read_text())
    for name, digest in expected.items():
        actual = hashlib.sha256()
        for seed in range(1, 1000):
            obstacle = resolve_scenario(config, name, seed)
            samples = [(position_at(obstacle, t), speed_at(obstacle, t))
                       for t in (0., .1, 1., 5., 6., 8., 10., 30., 60.)] if obstacle else []
            actual.update(json.dumps([obstacle, samples], sort_keys=True).encode())
        assert actual.hexdigest() == digest, name


def test_efficiency_scenes_clear_the_lane_and_recede_above_robot_speed():
    """Catch incomplete crossings, wall contact, slow leaders and endpoints blocking the goal."""
    config = yaml.safe_load(
        (Path(__file__).resolve().parents[1] /
         'config/simulation/scenarios.yaml').read_text())
    names = ['crossing_clear_' + level for level in ('early', 'mid', 'late')]
    assert all(name in config for name in names + ['receding'])
    for seed in range(1, 1000):
        crossings = [resolve_scenario(config, name, seed) for name in names]
        assert crossings[0]['delay'] < crossings[1]['delay'] < crossings[2]['delay']
        for obstacle in crossings:
            assert obstacle['x'] == 3.0
            assert 0. <= obstacle['y'] < .22
            assert .57 < position_at(obstacle, 60)[1] < 1.675
            assert speed_at(obstacle, 60) == 0
        leader = resolve_scenario(config, 'receding', seed)
        assert leader['x'] - .25 - .11 > .5 * leader['delay']
        for t in (leader['delay'], 1., 8.):
            assert speed_at(leader, t) > .5
        assert 7.4 < position_at(leader, 60)[0] < 9.0
        assert speed_at(leader, 60) == 0


def test_leaving_circle_never_approaches_a_robot_waiting_before_the_lane():
    """Reject a full crossing that can strike a stopped robot near the crossing point."""
    from simulation_metrics import clearance_to_circle

    config = yaml.safe_load(
        (Path(__file__).resolve().parents[1] /
         'config/simulation/scenarios.yaml').read_text())
    for level in ('early', 'mid', 'late'):
        for seed in (1, 4, 9):
            obstacle = resolve_scenario(config, 'crossing_clear_' + level, seed)
            for robot_x in (0., 1., 2., 2.4, 2.63):
                initial = clearance_to_circle(
                    robot_x, 0., 0., obstacle['x'], obstacle['y'], .25)
                assert initial > 0.
                for after_start in (0., .5, 1., 2., 3., 60.):
                    x, y = position_at(obstacle, obstacle['delay'] + after_start)
                    assert clearance_to_circle(robot_x, 0., 0., x, y, .25) >= initial


def test_scene_origin_rotates_seeded_motion_without_changing_random_draws():
    """Catch world-y randomization after rotation and missing heading transforms."""
    import math
    import pytest

    config = yaml.safe_load(
        (Path(__file__).resolve().parents[1] /
         'config/simulation/scenarios.yaml').read_text())
    original = resolve_scenario(config, 'oncoming', 8)
    moved = resolve_scenario(config, 'oncoming', 8, origin=[10., 20., math.pi / 2])
    assert moved['x'] == pytest.approx(10. - original['y'])
    assert moved['y'] == pytest.approx(27.)
    assert moved['yaw'] == pytest.approx(original['yaw'] + math.pi / 2)
    assert moved['segments'] == original['segments']
    assert moved['delay'] == original['delay']
    assert moved['speed_scale'] == original['speed_scale']
    for time in (0., 1., 3., 10.):
        x, y = position_at(original, time)
        assert position_at(moved, time) == pytest.approx((10. - y, 20. + x))
    assert resolve_scenario(config, 'oncoming', 8) == original
    assert config['oncoming']['x'] == 7.
    assert resolve_scenario(config, 'clear', 8, origin=[10., 20., math.pi / 2]) is None


def test_invalid_scene_origin_is_rejected():
    """Reject malformed placement before starting the simulator."""
    import pytest

    config = {'_variation': {'speed_scale': [1, 1], 'delay_offset_s': [0, 0]},
              'static': {'x': 3., 'y': 0., 'yaw': 0., 'delay': 0.}}
    for origin in ([1., 2.], [1., 2., float('nan')], [1., float('inf'), 0.]):
        with pytest.raises(ValueError):
            resolve_scenario(config, 'static', 8, origin=origin)


def test_simulation_spawns_resolved_pose_and_keeps_default_pose(monkeypatch):
    """Catch a mismatch between runner evaluation coordinates and Gazebo spawn."""
    import importlib.util
    import xml.etree.ElementTree as ET

    from launch import LaunchContext
    from launch.utilities import perform_substitutions
    import pytest

    root = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location(
        'simulation_launch', root / 'launch/simulation.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(root))
    for overrides, expected in [
            ({'x': '11', 'y': '27', 'yaw': '4.7'}, [11., 27., .64, 0., 0., 4.7]),
            ({'x': '', 'y': '', 'yaw': ''}, [7., 0., .64, 0., 0., 3.14159265359])]:
        context = LaunchContext()
        context.launch_configurations.update({
            'environment': str(root / 'config/simulation/corridor.yaml'),
            'mesh': '', 'scenario': 'oncoming', 'gui': 'false', 'run': 'false',
            **{key: '' for key in ('x', 'y', 'z', 'yaw')},
            **{'obstacle_' + key: value for key, value in overrides.items()},
        })
        actions = module.start_simulation(context)
        command = [perform_substitutions(context, part) for part in actions[1].cmd]
        tree = ET.parse(command[2])
        pose = tree.find("./world/model[@name='obstacle']/pose").text
        assert list(map(float, pose.split())) == pytest.approx(expected)


def test_motion_checker_uses_external_scene_origin(tmp_path, monkeypatch):
    """Catch false motion failures from comparing moved obstacles to corridor coordinates."""
    import csv
    import math

    import check_obstacle_motion

    root = Path(__file__).resolve().parents[1]
    monkeypatch.setattr(check_obstacle_motion, 'get_package_share_directory', lambda _: str(root))
    environment = tmp_path / 'environment.yaml'
    environment.write_text(yaml.safe_dump({'scenario_origin': [10., 20., math.pi / 2]}))
    result = {'scenario': 'static', 'method': 'mppi', 'seed': 8, 'output': str(tmp_path),
              'environment': str(environment), 'actual_delay_s': 0.,
              'actual_segments': '[{"speed": 0.0, "duration": 0.0}]'}
    with (tmp_path / 'results.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(result))
        writer.writeheader()
        writer.writerow(result)
    (tmp_path / 'samples.csv').write_text(
        'time_s,obstacle_x_m,obstacle_y_m\n0.0,10.0,23.0\n0.5,10.0,23.0\n1.0,10.0,23.0\n')
    assert check_obstacle_motion.check_batch(tmp_path)
