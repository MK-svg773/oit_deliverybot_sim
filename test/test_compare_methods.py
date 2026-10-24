"""Check paired inference against hand-calculated synthetic trial tables."""
import csv
from pathlib import Path
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
    success = next(r for r in tests if r['comparison'] == 'C1-C2'
                   and r['metric'] == 'success')
    assert (success['a_only'], success['c2_only']) == (0, 4)
    assert success['p_value'] == 0.125
    stopped = next(r for r in tests if r['comparison'] == 'C1-C2'
                   and r['metric'] == 'stopped_s')
    assert stopped['n'] == 6
    assert stopped['median_delta'] == 3.5
    assert stopped['statistic'] == 0
    assert stopped['p_value'] == 0.03125
    arrival = next(r for r in tests if r['comparison'] == 'C1-C2'
                   and r['metric'] == 'arrival_s')
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
    index.write_text('scenario,method,seed,retry_output\nclear,mppi,1,'
                     + str(retry.parent / 'trial') + '\n')
    loaded = compare.load_results(original, index)
    assert loaded[0]['outcome'] == 'success'
    assert loaded[1]['outcome'] == 'infrastructure_error'
