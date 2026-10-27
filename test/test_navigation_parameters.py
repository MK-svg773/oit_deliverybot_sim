"""Check method wiring and calibration artifacts without launching any processes."""
import importlib.util
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'navigation_launch', ROOT / 'launch/simulation_navigation.launch.py')
NAV = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(NAV)


def table(tmp_path, **changes):
    """Create a complete table for the configured 32-step horizon."""
    values = {'time_steps': 32, 'model_dt': .1, 'prediction_margins': [0.] * 32}
    values.update(changes)
    path = tmp_path / 'calibration.yaml'
    path.write_text(yaml.safe_dump(values))
    return str(path)


@pytest.mark.parametrize('method', ['dwb', 'mppi', 'mppi_pos', 'mppi_cv', 'mppi_age'])
def test_legacy_navigation_parameters_unchanged(method):
    """Catch C2u changing any existing controller parameter or enabling C0 tracking."""
    config = ROOT / 'config/simulation'
    expected = yaml.safe_load((config / 'nav2.yaml').read_text())
    if method.startswith('mppi'):
        expected['controller_server']['ros__parameters']['FollowPath'] = yaml.safe_load(
            (config / 'mppi.yaml').read_text())
    if method in ('mppi_pos', 'mppi_cv', 'mppi_age'):
        controller = expected['controller_server']['ros__parameters']['FollowPath']
        controller['critics'].append('TrackedObstacleCritic')
        controller['TrackedObstacleCritic'] = {
            'use_velocity': method != 'mppi_pos', 'cost_weight': 20.0, 'cost_power': 1,
            'safety_margin': .25, 'max_track_age': 1.2}
        if method == 'mppi_age':
            controller['TrackedObstacleCritic']['extrapolate_future'] = False
    # Baselines in a mixed run must ignore the C2u-only artifact, even if it is absent.
    assert NAV.navigation_parameters(config, method, .2, '/absent/table.yaml') == expected


def test_c2u_passes_only_margin_array_to_the_critic(tmp_path):
    """Prevent calibration truth/metadata from becoming runtime ROS parameters."""
    config = ROOT / 'config/simulation'
    path = table(tmp_path, calibration={'seeds': [100], 'truth': 'offline only'})
    expected = NAV.navigation_parameters(config, 'mppi_cv', .2)
    expected['controller_server']['ros__parameters']['FollowPath'][
        'TrackedObstacleCritic']['prediction_margins'] = [0.] * 32
    assert NAV.navigation_parameters(config, 'mppi_cvu', .2, path) == expected


@pytest.mark.parametrize('changes', [
    {'time_steps': 31}, {'model_dt': .2}, {'prediction_margins': []},
    {'prediction_margins': [-.1] * 32}, {'prediction_margins': [float('nan')] * 32},
    {'prediction_margins': [float('inf')] * 32}, {'prediction_margins': ['0.2'] * 32},
    {'prediction_margins': [True] * 32},
])
def test_invalid_calibration_fails_before_launch(tmp_path, changes):
    """Catch silent fallback, malformed tables and mismatched prediction times."""
    with pytest.raises(ValueError, match='calibration|prediction_margins'):
        NAV.navigation_parameters(ROOT / 'config/simulation', 'mppi_cvu', 0.,
                                  table(tmp_path, **changes))


def test_c2u_requires_a_table():
    """A named C2u run must not silently run ordinary C2."""
    with pytest.raises(ValueError, match='calibration_file'):
        NAV.navigation_parameters(ROOT / 'config/simulation', 'mppi_cvu', 0.)
