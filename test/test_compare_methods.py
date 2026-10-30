"""Check paired inference against hand-calculated synthetic trial tables."""
import csv
from pathlib import Path
import subprocess
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import compare_methods as compare  # noqa: E402, I100


def test_synthetic_csv_known_inference(tmp_path):
    """Catch reversed pairing, wrong denominators and wrong exact test tails."""
    fields = ['scenario', 'method', 'seed', 'outcome', 'arrival_s',
              'stopped_s', 'min_clearance_m', 'output']
    rows = []
    for seed in range(1, 7):
        for method in compare.METHODS:
            success = method == 'mppi_cv' or seed <= 2
            rows.append(dict(zip(fields, ['example', method, str(seed),
                                          'success' if success else 'collision',
                                          10 + seed if method == 'mppi_cv' else 10,
                                          seed if method == 'mppi_cv' else 0,
                                          0.1 * seed if method == 'mppi_cv' else 0,
                                          ''])))
    path = tmp_path / 'results.csv'
    with path.open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    loaded = compare.load_results(path)
    summary, tests = compare.analyze(loaded)
    c2 = next(r for r in summary if r['method'] == 'mppi_cv')
    assert c2['success'] == 6
    assert c2['success_rate'] == 1
    assert c2['wilson_low'] == pytest.approx(0.6096657)
    success = next(r for r in tests if r['comparison'] == 'C1-C2' and
                   r['metric'] == 'success')
    assert (success['a_only'], success['c2_only']) == (0, 4)
    assert success['p_value'] == 0.125
    stopped = next(r for r in tests if r['comparison'] == 'C1-C2' and
                   r['metric'] == 'stopped_s')
    assert stopped['n'] == 6
    assert stopped['median_delta'] == 3.5
    assert stopped['statistic'] == 0
    assert stopped['p_value'] == 0.03125
    arrival = next(r for r in tests if r['comparison'] == 'C1-C2' and
                   r['metric'] == 'arrival_s')
    assert arrival['n'] == 2
    assert arrival['median_delta'] == 1.5
    assert arrival['p_value'] == 0.5
    compare.write_outputs(loaded, summary, tests, tmp_path / 'analysis')
    assert (tmp_path / 'analysis/success_rates.png').is_file()
    assert (tmp_path / 'analysis/differences.png').is_file()


def test_zero_missing_and_holm():
    """Catch undefined all-zero tests and correction families that omit scenes."""
    assert compare.holm([0.01, 0.04, 0.03, None]) == [0.04, 0.09, 0.09, None]
    result = compare.continuous_test([0, 0, 0])
    assert result['p_value'] == 1
    assert result['ci_low'] == result['ci_high'] == 0
    assert compare.continuous_test([])['p_value'] is None
    result = compare.continuous_test([2])
    assert result['ci_low'] == result['ci_high'] == 2
    assert result['p_value'] == 1


def test_pairing_and_retry_validation(tmp_path):
    """Reject duplicate or missing seeds instead of silently unpaired analyses."""
    rows = [{'scenario': 'clear', 'method': method, 'seed': '1',
             'outcome': 'infrastructure_error', 'output': ''}
            for method in compare.METHODS]
    compare.analyze(rows)
    with pytest.raises(ValueError, match='Duplicate'):
        compare.analyze(rows + rows[:1])
    with pytest.raises(ValueError, match='seed'):
        compare.analyze(rows[:-1])
    fields = list(rows[0])
    original = tmp_path / 'original/results.csv'
    retry = tmp_path / 'retry/results.csv'
    for path, data in [(original, rows), (retry, [dict(rows[0], outcome='success',
                                                       output=str(retry.parent / 'trial'))])]:
        path.parent.mkdir()
        with path.open('w') as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(data)
    index = tmp_path / 'retry_index.csv'
    index.write_text('scenario,method,seed,retry_output\nclear,mppi,1,' +
                     str(retry.parent / 'trial') + '\n')
    loaded = compare.load_results(original, index)
    assert loaded[0]['outcome'] == 'success'
    assert loaded[1]['outcome'] == 'infrastructure_error'


def age_rows():
    """Use six paired seeds in five scenes with known, unequal method effects."""
    rows = []
    for scene in ('crossing', 'following', 'oncoming', 'stop_go', 'speed_change'):
        for seed in range(1, 7):
            for method, successes, stopped in (
                    ('mppi_pos', 2, 10), ('mppi_age', 4, 10 + seed),
                    ('mppi_cv', 6, 10 + 3 * seed)):
                rows.append({
                    'scenario': scene, 'method': method, 'seed': str(seed),
                    'outcome': 'success' if seed <= successes else 'collision',
                    'arrival_s': str(stopped), 'stopped_s': str(stopped),
                    'min_clearance_m': str(stopped / 100), 'output': '',
                })
    return rows


def test_age_pairing_direction_and_holm():
    """Separate the age and future effects without requiring C0 rows."""
    rows = age_rows()
    summary, tests = compare.analyze(rows, comparisons=('C1-C_age', 'C_age-C2'))
    assert {r['method'] for r in summary} == {'mppi_pos', 'mppi_age', 'mppi_cv'}
    for label, delta, arrival_n, direction in (
            ('C1-C_age', 3.5, 2, 'mppi_age - mppi_pos'),
            ('C_age-C2', 7., 4, 'mppi_cv - mppi_age')):
        group = {r['metric']: r for r in tests
                 if r['scenario'] == 'following' and r['comparison'] == label}
        assert (group['success']['a_only'], group['success']['b_only']) == (0, 2)
        assert (group['collision']['a_only'], group['collision']['b_only']) == (2, 0)
        assert group['success']['p_value'] == .5
        assert group['stopped_s']['median_delta'] == delta
        assert group['stopped_s']['delta_direction'] == direction
        assert group['stopped_s']['n'] == 6
        assert group['stopped_s']['p_value'] == .03125
        assert group['stopped_s']['p_holm'] == .15625  # Five scenes, per comparison/metric.
        assert group['arrival_s']['n'] == arrival_n
    with pytest.raises(ValueError, match='seed'):
        compare.analyze(rows[:-1], comparisons=('C1-C_age', 'C_age-C2'))
    with pytest.raises(ValueError, match='Duplicate'):
        compare.analyze(rows + rows[:1], comparisons=('C1-C_age', 'C_age-C2'))


def test_calibrated_margin_comparisons():
    """Pair C2u against C2 and C1 with C2u as the delta target."""
    rows = [dict(r, method={'mppi_age': 'mppi_cvu', 'mppi_cv': 'mppi_cv'}.get(
        r['method'], r['method'])) for r in age_rows()]
    _, tests = compare.analyze(rows, comparisons=('C2-C2u', 'C1-C2u'))
    assert {r['delta_direction'] for r in tests} == {
        'mppi_cvu - mppi_cv', 'mppi_cvu - mppi_pos'}


def test_age_comparison_cli_outputs(tmp_path):
    """Exercise selection, CSV direction labels and plots through the real CLI."""
    source = tmp_path / 'input.csv'
    compare.write_csv(source, age_rows())
    output = tmp_path / 'analysis'
    result = subprocess.run([
        sys.executable, str(Path(compare.__file__)), str(source), '--output', str(output),
        '--comparisons', 'C1-C_age', 'C_age-C2'], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    tests = compare.read_csv(output / 'tests.csv')
    assert {r['delta_direction'] for r in tests} == {
        'mppi_age - mppi_pos', 'mppi_cv - mppi_age'}
    assert 'c2_only' not in tests[0]
    assert 'b_only' in tests[0]
    notes = (output / 'tables.md').read_text()
    assert 'C_age−C1' in notes and 'C2−C_age' in notes
    assert (output / 'success_rates.png').is_file()
    assert (output / 'differences.png').is_file()


def monitor_rows():
    """Use the same method/seed on both sides of the intervention."""
    rows = []
    for scene in ('crossing', 'following'):
        for seed in range(1, 7):
            for method, enabled, successes, stopped in (
                    ('mppi', '', 1, 10), ('mppi', 'true', 3, 10 + seed),
                    ('mppi_pos', 'true', 2, 10 + seed),
                    ('mppi_cv', 'false', 2, 10), ('mppi_cv', 'true', 6, 10 + 3 * seed)):
                rows.append({
                    'scenario': scene, 'method': method, 'seed': str(seed),
                    'collision_monitor': enabled,
                    'outcome': 'success' if seed <= successes else 'collision',
                    'arrival_s': str(stopped), 'stopped_s': str(stopped),
                    'min_clearance_m': str(stopped / 100), 'output': '',
                })
    return rows


def test_monitor_pairing_direction_and_holm():
    """Do not collapse CM on/off trials or pair CM C1 with baseline C2."""
    rows = monitor_rows()
    labels = ('C0-C0cm', 'C2-C2cm', 'C1cm-C2cm')
    summary, tests = compare.analyze(rows, comparisons=labels)
    assert {r['method'] for r in summary} == {
        'mppi', 'mppi_cm', 'mppi_pos_cm', 'mppi_cv', 'mppi_cv_cm'}
    for label, wins, delta, arrival_n in (
            ('C0-C0cm', 2, 3.5, 1), ('C2-C2cm', 4, 10.5, 2),
            ('C1cm-C2cm', 4, 7., 2)):
        group = {r['metric']: r for r in tests
                 if r['scenario'] == 'crossing' and r['comparison'] == label}
        assert (group['success']['a_only'], group['success']['b_only']) == (0, wins)
        assert group['stopped_s']['median_delta'] == delta
        assert group['stopped_s']['p_holm'] == .0625  # Two scenes per comparison/metric.
        assert group['arrival_s']['n'] == arrival_n
    with pytest.raises(ValueError, match='Duplicate'):
        compare.analyze(rows + rows[:1], comparisons=labels)
    with pytest.raises(ValueError, match='seed'):
        compare.analyze(rows[:-1], comparisons=labels)
    assert compare.key(rows[0]) == compare.key(
        {k: v for k, v in rows[0].items() if k != 'collision_monitor'})


def test_monitor_comparison_cli_preserves_recorded_methods(tmp_path):
    """Accept combined historical/new CSVs without relabeling recorded methods."""
    source = tmp_path / 'input.csv'
    rows = monitor_rows()
    compare.write_csv(source, rows)
    output = tmp_path / 'analysis'
    result = subprocess.run([
        sys.executable, str(Path(compare.__file__)), str(source), '--output', str(output),
        '--comparisons', 'C0-C0cm', 'C2-C2cm', 'C1cm-C2cm'], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert compare.read_csv(output / 'results.csv') == rows
    tests = compare.read_csv(output / 'tests.csv')
    assert {r['delta_direction'] for r in tests} == {
        'mppi_cm - mppi', 'mppi_cv_cm - mppi_cv', 'mppi_cv_cm - mppi_pos_cm'}
    assert (output / 'success_rates.png').is_file()


def test_monitor_retry_replaces_only_matching_intervention(tmp_path):
    """A CM retry must not overwrite the baseline with the same seed and method."""
    rows = [dict(r, outcome='infrastructure_error') for r in monitor_rows()[:2]]
    original = tmp_path / 'original.csv'
    compare.write_csv(original, rows)
    retry = tmp_path / 'retry'
    retry.mkdir()
    compare.write_csv(retry / 'results.csv', [dict(
        rows[1], outcome='success', output=str(retry / 'trial'))])
    index = tmp_path / 'retry_index.csv'
    index.write_text('scenario,method,seed,collision_monitor,retry_output\n'
                     'crossing,mppi,1,true,' + str(retry / 'trial') + '\n')
    loaded = compare.load_results(original, index)
    assert [r['outcome'] for r in loaded] == ['infrastructure_error', 'success']
