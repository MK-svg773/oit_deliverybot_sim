#!/usr/bin/env python3
"""Calibrate empirical CV margins from development trial CSVs, never from schedules."""
import argparse
from bisect import bisect_left
import csv
import math
from pathlib import Path

import numpy as np
from simulation_metrics import interpolate_truth
import yaml


def read_csv(path):
    """Read an existing monitor/results file without modifying it."""
    with path.open() as stream:
        return list(csv.DictReader(stream))


def recorded_truth(samples, target, max_gap):
    """Use only finite, fresh bracketing recorded positions, never extrapolation."""
    history, times = samples
    i = bisect_left(times, target)
    if i < len(times) and times[i] == target:
        point = history[i][1:]
    else:
        point = interpolate_truth(history[max(0, i - 1):i + 1], target, max_gap)
    if not all(math.isfinite(v) for v in point):
        raise ValueError('Missing obstacle truth')
    return point


def trial_residuals(directory, offsets, max_age, max_gap):
    """Compare noisy published CV outputs at monitor processing time with future truth."""
    rows = read_csv(directory / 'samples.csv')
    # Keep invalid positions in the history so interpolation cannot bridge across missing data.
    history = [(float(row['time_s']), float(row['obstacle_x_m'] or 'nan'),
                float(row['obstacle_y_m'] or 'nan')) for row in rows]
    times = [row[0] for row in history]
    if (not all(math.isfinite(t) for t in times) or
            any(b <= a for a, b in zip(times, times[1:]))):
        raise ValueError(f'{directory}: truth timestamps must be finite and strictly increasing')
    residuals = [[] for _ in offsets]
    stats = {'track_rows': 0, 'unmatched_rows': 0, 'invalid_rows': 0, 'stale_or_future_rows': 0,
             'false_tracks': 0, 'missing_truth_counts': [0] * len(offsets)}
    for row in read_csv(directory / 'tracking_errors.csv'):
        stats['track_rows'] += 1
        stats['false_tracks'] += int(row['false_tracks'])
        if row['matched'] not in ('True', 'False'):
            raise ValueError(f'{directory}: invalid matched flag')
        if row['matched'] == 'False':
            stats['unmatched_rows'] += 1
            continue
        values = [float(row[key] or 'nan') for key in (
            'time_s', 'age_s', 'x_m', 'y_m', 'vx_mps', 'vy_mps')]
        if not all(math.isfinite(v) for v in values) or values[0] < 0:
            stats['invalid_rows'] += 1
            continue
        stamp, age, x, y, vx, vy = values
        if not 0 <= age <= max_age:
            stats['stale_or_future_rows'] += 1
            continue
        for j, offset in enumerate(offsets):
            elapsed = age + offset
            try:
                tx, ty = recorded_truth((history, times), stamp + elapsed, max_gap)
            except ValueError:
                stats['missing_truth_counts'][j] += 1
                continue
            residual = math.hypot(x + vx * elapsed - tx, y + vy * elapsed - ty)
            if not math.isfinite(residual):
                raise ValueError(f'{directory}: nonfinite prediction residual')
            residuals[j].append(residual)
    stats['sample_counts'] = [len(values) for values in residuals]
    return residuals, stats


def calibrate(result_paths, *, quantile, time_steps, model_dt, max_truth_gap=.12):
    """Pool time-indexed residuals for one development noise/delay/environment condition."""
    if quantile is None or not math.isfinite(quantile) or not 0 < quantile <= 1:
        raise ValueError('quantile must be explicitly supplied in (0, 1]')
    if (not isinstance(time_steps, int) or time_steps < 1 or
            not math.isfinite(model_dt) or model_dt <= 0):
        raise ValueError('time_steps and model_dt must be positive and finite')
    if not math.isfinite(max_truth_gap) or max_truth_gap <= 0:
        raise ValueError('max_truth_gap must be positive and finite')
    # Match C++ step * float model_dt before promotion to double (first step is k=1).
    offsets = [float(np.float32(k) * np.float32(model_dt))
               for k in range(1, time_steps + 1)]
    if not all(math.isfinite(t) and t > 0 for t in offsets):
        raise ValueError('model_dt produces an invalid float prediction horizon')
    pooled = [[] for _ in offsets]
    trials, skipped = [], []
    seen_paths, seen_trials, seeds, scenarios = set(), set(), set(), set()
    condition = None
    for result_path in result_paths:
        result_path = Path(result_path).expanduser().resolve()
        for row in read_csv(result_path):
            seed = int(row['seed'])
            if row['seed_set'] != 'development' or not 1 <= seed <= 999:
                raise ValueError('Only development seeds 1–999 may be used for calibration')
            directory = Path(row['output']).expanduser()
            directory = (directory if directory.is_absolute()
                         else result_path.parent / directory).resolve()
            noise, delay = float(row['velocity_noise_std_mps']), float(row['additional_delay_s'])
            if not all(math.isfinite(v) and v >= 0 for v in (noise, delay)):
                raise ValueError('Invalid noise/delay metadata')
            key = (row['scenario'], row['method'], seed, noise, delay)
            if directory in seen_paths or key in seen_trials:
                raise ValueError(f'Duplicate calibration trial: {directory}')
            seen_paths.add(directory)
            seen_trials.add(key)
            if row['outcome'] in ('infrastructure_error', 'cancelled'):
                skipped.append({'output': str(directory), 'seed': seed, 'outcome': row['outcome']})
                continue
            current = (noise, delay, row.get('environment', ''))
            if condition is not None and current != condition:
                raise ValueError(
                    'Use separate tables for different noise/delay/environment conditions')
            condition = current
            residuals, stats = trial_residuals(directory, offsets, delay + 1., max_truth_gap)
            for pool, values in zip(pooled, residuals):
                pool.extend(values)
            if any(stats['sample_counts']):
                seeds.add(seed)
                scenarios.add(row['scenario'])
            trials.append(dict(output=str(directory), seed=seed, scenario=row['scenario'],
                               method=row['method'], outcome=row['outcome'], **stats))
    if not all(pooled):
        missing = [j + 1 for j, values in enumerate(pooled) if not values]
        raise ValueError(f'No residual samples for prediction steps {missing}')
    margins = [float(np.quantile(values, quantile, method='inverted_cdf')) for values in pooled]
    totals = {key: sum(trial[key] for trial in trials) for key in (
        'track_rows', 'unmatched_rows', 'invalid_rows', 'stale_or_future_rows', 'false_tracks')}
    return {
        'time_steps': time_steps, 'model_dt': float(model_dt), 'prediction_margins': margins,
        'calibration': {
            'quantile': float(quantile), 'quantile_method': 'inverted_cdf',
            'interpretation': 'empirical marginal residual quantiles; no safety guarantee',
            'prediction_origin': 'tracking_errors.time_s + age_s (monitor processing time)',
            'seeds': sorted(seeds), 'scenarios': sorted(scenarios),
            'velocity_noise_std_mps': condition[0], 'additional_delay_s': condition[1],
            'environment': condition[2], 'max_truth_gap_s': max_truth_gap,
            'sample_counts': [len(values) for values in pooled],
            'missing_truth_counts': [sum(trial['missing_truth_counts'][j] for trial in trials)
                                     for j in range(time_steps)],
            **totals, 'trials': trials, 'skipped_trials': skipped}}


def main():
    """Write one launch-consumable YAML with an explicit empirical quantile."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('results', type=Path, nargs='+', help='Development results.csv files')
    parser.add_argument('--quantile', type=float, required=True, help='Explicit value in (0, 1]')
    parser.add_argument('--time-steps', type=int, required=True, help='MPPI time_steps')
    parser.add_argument('--model-dt', type=float, required=True, help='MPPI model_dt, seconds')
    parser.add_argument('--max-truth-gap', type=float, default=.12,
                        help='Maximum bracketing truth gap, seconds (same as monitor)')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    try:
        table = calibrate(args.results, quantile=args.quantile, time_steps=args.time_steps,
                          model_dt=args.model_dt, max_truth_gap=args.max_truth_gap)
    except (ValueError, KeyError, OSError) as exc:
        parser.error(str(exc))
    args.output.write_text(yaml.safe_dump(table, sort_keys=False))
    info = table['calibration']
    print(f"Wrote {args.output}; quantile={args.quantile}; seeds={info['seeds']}; "
          f"sample_counts={info['sample_counts']}")


if __name__ == '__main__':
    main()
