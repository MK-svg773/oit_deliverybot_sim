"""Exercise recorded-output calibration without starting ROS or a simulator."""
import csv
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from calibrate_prediction_margin import calibrate  # noqa: E402, I100


def write_csv(path, rows):
    """Write the same header/row representation as the experiment monitor."""
    with path.open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def trial(tmp_path, seed=100, seed_set='development', noise=.5, tracks=None,
          samples=None, outcome='success'):
    """Build one recorded trial with deliberately wrong scheduled truth columns."""
    directory = tmp_path / str(seed)
    directory.mkdir(exist_ok=True)
    if tracks is None:
        tracks = [{'time_s': 0., 'age_s': .2, 'matched': True, 'track_count': 1, 'false_tracks': 0,
                   'x_m': 0., 'y_m': 0., 'vx_mps': 2., 'vy_mps': 0.}]
    if samples is None:
        samples = [{'time_s': i * .05, 'obstacle_x_m': i * .05, 'obstacle_y_m': 0.,
                    'expected_obstacle_x_m': 900., 'expected_obstacle_y_m': 900.}
                   for i in range(41)]
    write_csv(directory / 'tracking_errors.csv', tracks)
    write_csv(directory / 'samples.csv', samples)
    results = directory / 'results.csv'
    write_csv(results, [{'seed': seed, 'seed_set': seed_set, 'scenario': 'following',
                         'method': 'mppi_cv', 'velocity_noise_std_mps': noise,
                         'additional_delay_s': 0., 'outcome': outcome,
                         'output': str(directory), 'duration_s': 2.}])
    return results


def test_residual_uses_observation_age_noisy_output_and_future_recorded_truth(tmp_path):
    """Catch omitted age, k=0, truth velocity substitution and scheduled labels."""
    result = calibrate([trial(tmp_path)], quantile=.9, time_steps=2, model_dt=.1)
    offsets = [float(np.float32(k) * np.float32(.1)) for k in (1, 2)]
    assert result['prediction_margins'] == pytest.approx([.2 + t for t in offsets])
    assert result['calibration']['sample_counts'] == [1, 1]
    assert result['calibration']['seeds'] == [100]


def test_empirical_quantile_and_seed_union_include_failed_trials(tmp_path):
    """Catch wrong quantile axis and silently excluding collisions/timeouts."""
    files = []
    for seed, offset, outcome in [(102, .2, 'collision'), (100, .8, 'timeout'),
                                  (101, .4, 'success')]:
        tracks = [{'time_s': 0., 'age_s': 0., 'matched': True, 'track_count': 1, 'false_tracks': 0,
                   'x_m': offset, 'y_m': 0., 'vx_mps': 1., 'vy_mps': 0.}]
        files.append(trial(tmp_path, seed=seed, tracks=tracks, outcome=outcome))
    result = calibrate(files, quantile=.5, time_steps=2, model_dt=.1)
    assert result['prediction_margins'] == pytest.approx([.4, .4])
    assert result['calibration']['seeds'] == [100, 101, 102]
    assert result['calibration']['sample_counts'] == [3, 3]
    assert result['calibration']['quantile_method'] == 'inverted_cdf'
    result = calibrate(files, quantile=.9, time_steps=2, model_dt=.1)
    assert result['prediction_margins'] == pytest.approx([.8, .8])


def test_missing_labels_unmatched_stale_and_invalid_tracks_are_counted(tmp_path):
    """Catch extrapolation past the end, bridging gaps, or treating NaN as zero."""
    base = {'time_s': 0., 'age_s': 0., 'matched': True, 'track_count': 1, 'false_tracks': 0,
            'x_m': 0., 'y_m': 0., 'vx_mps': 1., 'vy_mps': 0.}
    tracks = [base, dict(base, time_s=1.85), dict(base, time_s=.6),
              dict(base, matched=False, x_m='', vx_mps='', track_count=0),
              dict(base, vx_mps='nan'), dict(base, age_s=1.01), dict(base, age_s=-.1)]
    samples = [{'time_s': i * .05, 'obstacle_x_m': i * .05, 'obstacle_y_m': 0.}
               for i in range(41) if not 11 <= i <= 18]
    result = calibrate([trial(tmp_path, tracks=tracks, samples=samples)],
                       quantile=.9, time_steps=2, model_dt=.1)
    info = result['calibration']
    assert info['sample_counts'] == [2, 1]
    assert info['missing_truth_counts'] == [1, 2]
    assert info['unmatched_rows'] == 1
    assert info['invalid_rows'] == 1
    assert info['stale_or_future_rows'] == 2


@pytest.mark.parametrize('quantile', [None, 0., 1.1, float('nan')])
def test_quantile_is_explicit_and_valid(tmp_path, quantile):
    """Prevent implicit confidence choices and nonfinite quantile output."""
    with pytest.raises(ValueError, match='quantile'):
        calibrate([trial(tmp_path)], quantile=quantile, time_steps=2, model_dt=.1)


@pytest.mark.parametrize('steps,dt', [(0, .1), (2, 0.), (2, float('nan'))])
def test_invalid_horizon_is_rejected(tmp_path, steps, dt):
    """Reject tables with no meaningful prediction horizon."""
    with pytest.raises(ValueError, match='time_steps|model_dt'):
        calibrate([trial(tmp_path)], quantile=.9, time_steps=steps, model_dt=dt)


def test_final_seeds_and_mixed_noise_are_rejected(tmp_path):
    """Keep held-out labels and incompatible noise conditions out of calibration."""
    final = trial(tmp_path, seed=10000, seed_set='final')
    with pytest.raises(ValueError, match='development'):
        calibrate([final], quantile=.9, time_steps=2, model_dt=.1)
    files = [trial(tmp_path, seed=100, noise=0.), trial(tmp_path, seed=101, noise=.5)]
    with pytest.raises(ValueError, match='noise|delay'):
        calibrate(files, quantile=.9, time_steps=2, model_dt=.1)


def test_no_samples_for_any_step_is_an_error(tmp_path):
    """Do not silently fill unsupported long horizons with zero margins."""
    with pytest.raises(ValueError, match='samples'):
        calibrate([trial(tmp_path)], quantile=.9, time_steps=32, model_dt=.1)


def test_duplicate_trials_are_rejected(tmp_path):
    """Prevent duplicated paths from giving a trial extra calibration weight."""
    results = trial(tmp_path)
    with pytest.raises(ValueError, match='Duplicate'):
        calibrate([results, results], quantile=.9, time_steps=2, model_dt=.1)


def test_cli_writes_double_array_and_metadata_requires_quantile(tmp_path):
    """Check the artifact consumed by launch and the required CLI argument."""
    script = Path(__file__).resolve().parents[1] / 'scripts/calibrate_prediction_margin.py'
    output = tmp_path / 'margin.yaml'
    args = [sys.executable, str(script), str(trial(tmp_path)), '--time-steps', '2',
            '--model-dt', '.1', '--output', str(output)]
    missing = subprocess.run(args, capture_output=True, text=True)
    assert missing.returncode != 0
    assert '--quantile' in missing.stderr
    assert not output.exists()
    done = subprocess.run([*args, '--quantile', '.9'], capture_output=True, text=True)
    assert done.returncode == 0, done.stderr
    table = yaml.safe_load(output.read_text())
    assert table['model_dt'] == .1 and table['time_steps'] == 2
    assert all(isinstance(value, float) for value in table['prediction_margins'])
    assert table['calibration']['seeds'] == [100]
    assert table['calibration']['quantile'] == .9
    done = subprocess.run([*args, '--quantile', '.9', '--directional'],
                          capture_output=True, text=True)
    assert done.returncode == 0, done.stderr
    directional = yaml.safe_load(output.read_text())
    assert directional['prediction_margins'] == table['prediction_margins']
    # Default fixture overpredicts eastward motion; only the westward score is positive.
    assert directional['directional_prediction_margins'][0][0] == 0.
    assert directional['directional_prediction_margins'][0][18] == pytest.approx(.3)


def test_directional_quantiles_rotate_residuals_and_exclude_slow_tracks(tmp_path):
    """Catch wrong residual sign/frame, interpolated quantiles and slow-row inclusion."""
    files = []
    # Velocity points north: world residual (-2, a) is local (a, 2).
    # Nine a=1 and one a=10 give inverted-CDF Q_0.9=1, not 1.9.
    for i, a in enumerate([1.] * 9 + [10.]):
        tracks = [{'time_s': 0., 'age_s': .2, 'matched': True, 'false_tracks': 0,
                   'x_m': 2., 'y_m': -a, 'vx_mps': 0., 'vy_mps': 1.}]
        samples = [{'time_s': k * .05, 'obstacle_x_m': 0., 'obstacle_y_m': k * .05}
                   for k in range(41)]
        files.append(trial(tmp_path, seed=130 + i, tracks=tracks, samples=samples))
    tracks = [{'time_s': 0., 'age_s': 0., 'matched': True, 'false_tracks': 0,
               'x_m': -100., 'y_m': 0., 'vx_mps': .099, 'vy_mps': 0.}]
    files.append(trial(tmp_path, seed=140, tracks=tracks))
    old = calibrate(files, quantile=.9, time_steps=2, model_dt=.1)
    result = calibrate(files, quantile=.9, time_steps=2, model_dt=.1, directional=True)
    assert result['prediction_margins'] == old['prediction_margins']
    assert result['calibration']['sample_counts'] == [11, 11]
    assert result['calibration']['directional_sample_counts'] == [10, 10]
    assert result['calibration']['low_speed_rows'] == 1
    for row in result['directional_prediction_margins']:
        assert len(row) == 36
        assert [row[b] for b in (0, 9, 18, 27)] == pytest.approx([1., 2., 0., 0.])
        assert row[3] == pytest.approx(3 ** .5 / 2 + 1.)


def test_directional_zero_point_one_speed_is_included(tmp_path):
    """The fixed lower bound is inclusive, while no directional data must fail."""
    tracks = [{'time_s': 0., 'age_s': 0., 'matched': True, 'false_tracks': 0,
               'x_m': 0., 'y_m': 0., 'vx_mps': .1, 'vy_mps': 0.}]
    source = trial(tmp_path, tracks=tracks)
    result = calibrate([source], quantile=.9, time_steps=2, model_dt=.1, directional=True)
    assert result['calibration']['directional_sample_counts'] == [1, 1]
    tracks[0]['vx_mps'] = .099
    source = trial(tmp_path, tracks=tracks)
    with pytest.raises(ValueError, match='directional.*samples'):
        calibrate([source], quantile=.9, time_steps=2, model_dt=.1, directional=True)
