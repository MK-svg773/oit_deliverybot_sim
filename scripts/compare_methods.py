#!/usr/bin/env python3
"""Compare paired C0/C1/C2 trials without excluding unsuccessful navigation."""
import argparse
from collections import Counter
import csv
from pathlib import Path
import re

import matplotlib
import numpy as np
from scipy import stats

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402, I100

METHODS = ('mppi', 'mppi_pos', 'mppi_cv')
COMPARISONS = (('C1-C2', 'mppi_pos'), ('C0-C2', 'mppi'))
METRICS = ('success', 'collision', 'arrival_s', 'stopped_s', 'min_clearance_m')


def read_csv(path):
    """Read trial or replacement dictionaries."""
    with Path(path).open() as stream:
        return list(csv.DictReader(stream))


def key(row):
    """Pair by scenario, method and seed rather than trial number."""
    return row['scenario'], row['method'], int(row['seed'])


def load_results(path, replacements=None):
    """Apply explicit one-time infrastructure retries using their batch CSVs."""
    rows = read_csv(path)
    if replacements is None:
        return rows
    mapping = {key(r): r for r in rows}
    if len(mapping) != len(rows):
        raise ValueError('Duplicate trial keys')
    seen = set()
    for replacement in read_csv(replacements):
        trial_key = key(replacement)
        if trial_key in seen:
            raise ValueError('Duplicate retry key')
        seen.add(trial_key)
        old = mapping[trial_key]
        retry_output = Path(replacement['retry_output'])
        if old['output'] == str(retry_output):
            continue  # Allow an already substituted CSV with its provenance index.
        if old['outcome'] != 'infrastructure_error':
            raise ValueError('Only infrastructure_error may be replaced')
        candidates = [r for r in read_csv(retry_output.parent / 'results.csv')
                      if key(r) == trial_key and r['output'] == str(retry_output)]
        if len(candidates) != 1:
            raise ValueError(f'Retry row not unique: {trial_key}')
        mapping[trial_key] = candidates[0]
    return [mapping[key(r)] for r in rows]


def holm(p_values):
    """Correct one comparison/metric family across all scenes, including missing."""
    ordered = sorted(range(len(p_values)),
                     key=lambda i: 1 if p_values[i] is None else p_values[i])
    adjusted = [None] * len(p_values)
    previous = 0.
    for rank, i in enumerate(ordered):
        value = 1. if p_values[i] is None else p_values[i]
        previous = max(previous, min(1., (len(p_values) - rank) * value))
        if p_values[i] is not None:
            adjusted[i] = previous
    return adjusted


def continuous_test(differences):
    """Test C2 minus comparator; percentile bootstrap of paired median differences."""
    d = np.round(np.asarray(differences, dtype=float), 12)
    if not len(d):
        return {
            'n': 0,
            'median_delta': None,
            'ci_low': None,
            'ci_high': None,
            'statistic': None,
            'p_value': None,
            'test_method': 'no finite pairs',
        }
    median = float(np.median(d))
    if len(d) == 1:
        low = high = median
    else:
        ci = stats.bootstrap((d,), np.median, method='percentile',
                             n_resamples=10000, random_state=0).confidence_interval
        low, high = float(ci.low), float(ci.high)
    if np.all(d == 0):
        statistic, p_value, method = 0., 1., 'all zero'
    else:
        # SciPy 1.11 exact distribution assumes no zero differences or ties.
        method = ('exact' if np.all(d != 0) and len(set(abs(d))) == len(d)
                  else 'approx')
        result = stats.wilcoxon(d, zero_method='wilcox', method=method,
                                alternative='two-sided', correction=False)
        statistic, p_value = float(result.statistic), float(result.pvalue)
    return {
        'n': len(d),
        'median_delta': median,
        'ci_low': low,
        'ci_high': high,
        'statistic': statistic,
        'p_value': p_value,
        'test_method': method,
    }


def analyze(rows):
    """Summarize all outcomes and test seed-matched scenario pairs."""
    indexed = {key(r): r for r in rows}
    if len(indexed) != len(rows):
        raise ValueError('Duplicate scenario/method/seed')
    scenes = list(dict.fromkeys(r['scenario'] for r in rows))
    summary, tests = [], []
    for scene in scenes:
        seeds = {int(r['seed']) for r in rows if r['scenario'] == scene}
        for method in METHODS:
            group = [r for r in rows if r['scenario'] == scene and r['method'] == method]
            if {int(r['seed']) for r in group} != seeds:
                raise ValueError(f'Unmatched seed sets: {scene}/{method}')
            counts = Counter(r['outcome'] for r in group)
            ci = stats.binomtest(counts['success'], len(group)).proportion_ci(method='wilson')
            summary.append({
                'scenario': scene,
                'method': method,
                'n': len(group),
                'success': counts['success'],
                'collision': counts['collision'],
                'timeout': counts['timeout'],
                'infrastructure_error': counts['infrastructure_error'],
                'other': len(group) - sum(counts[k] for k in
                                          ('success', 'collision', 'timeout',
                                           'infrastructure_error')),
                'success_rate': counts['success'] / len(group),
                'wilson_low': ci.low,
                'wilson_high': ci.high,
            })
        for label, method in COMPARISONS:
            pairs = [(indexed[scene, method, seed], indexed[scene, 'mppi_cv', seed])
                     for seed in sorted(seeds)]
            for metric in METRICS:
                result = {
                    'scenario': scene,
                    'comparison': label,
                    'metric': metric,
                    'pairs_total': len(pairs),
                    'n': 0,
                    'missing_pairs': 0,
                    'a_only': None,
                    'c2_only': None,
                    'median_delta': None,
                    'ci_low': None,
                    'ci_high': None,
                    'statistic': None,
                    'p_value': None,
                    'p_holm': None,
                    'test_method': '',
                }
                if metric in ('success', 'collision'):
                    a_only = sum(a['outcome'] == metric and b['outcome'] != metric
                                 for a, b in pairs)
                    b_only = sum(a['outcome'] != metric and b['outcome'] == metric
                                 for a, b in pairs)
                    discordant = a_only + b_only
                    result.update(n=len(pairs), a_only=a_only, c2_only=b_only,
                                  statistic=min(a_only, b_only),
                                  p_value=(stats.binomtest(a_only, discordant).pvalue
                                           if discordant else 1.),
                                  test_method='exact McNemar')
                else:
                    differences = []
                    for a, b in pairs:
                        if 'infrastructure_error' in (a['outcome'], b['outcome']):
                            continue
                        if metric == 'arrival_s' and not (
                                a['outcome'] == b['outcome'] == 'success'):
                            continue
                        if not a.get(metric) or not b.get(metric):
                            continue
                        values = float(a[metric]), float(b[metric])
                        if np.isfinite(values).all():
                            differences.append(values[1] - values[0])
                    result.update(continuous_test(differences))
                    result['missing_pairs'] = len(pairs) - result['n']
                tests.append(result)
    for label, _ in COMPARISONS:
        for metric in METRICS:
            family = [r for r in tests if r['comparison'] == label and r['metric'] == metric]
            for row, corrected in zip(family, holm([r['p_value'] for r in family])):
                row['p_holm'] = corrected
    return summary, tests


def table(rows):
    """Render CSV columns as an auditable Markdown table."""
    fields = list(rows[0])
    lines = ['|' + '|'.join(fields) + '|', '|' + '|'.join(['---'] * len(fields)) + '|']
    for row in rows:
        values = [('—' if row[k] is None else f'{row[k]:.6g}'
                   if isinstance(row[k], (float, np.floating)) else str(row[k]))
                  for k in fields]
        lines.append('|' + '|'.join(values) + '|')
    return '\n'.join(lines)


def write_csv(path, rows):
    """Write numeric output at full precision."""
    with path.open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def write_outputs(rows, summary, tests, output):
    """Write inference, plots and existing controller timing evidence."""
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    write_csv(output / 'results.csv', rows)
    write_csv(output / 'outcomes.csv', summary)
    write_csv(output / 'tests.csv', tests)
    notes = ('差はC2−比較方式。到達時間は両方成功、停止時間・距離は失敗を含む全有限組。'
             '基盤異常の数値指標と欠測は補完せずmissing_pairsに記載。成否の分母は全組。\n'
             'McNemarは両側正確二項検定。不一致a_onlyは比較方式だけ該当、c2_onlyはC2だけ該当。\n'
             'Wilcoxonは両側・wilcox・連続補正なし、ゼロ/同順位ありは正規近似、他はexact。'
             '差を小数12桁に丸め、全ゼロはp=1、組数0は欠測。\n'
             '中央値95%区間は対応差を10000回再標本化するpercentile bootstrap（乱数0）。'
             '1組の区間は退化し推論には不十分。\n'
             'Holmは比較方式×指標ごとに全場面数で補正。欠測場面もp=1相当で枠を保持。\n')
    (output / 'tables.md').write_text(notes + '\n' + table(summary) + '\n\n' + table(tests) + '\n')
    scenes = list(dict.fromkeys(r['scenario'] for r in summary))
    fig, ax = plt.subplots(figsize=(11, 5), layout='constrained')
    for j, method in enumerate(METHODS):
        values = [next(r for r in summary if r['scenario'] == s and r['method'] == method)
                  for s in scenes]
        rates = np.array([r['success_rate'] for r in values])
        low = np.array([r['wilson_low'] for r in values])
        high = np.array([r['wilson_high'] for r in values])
        ax.errorbar(np.arange(len(scenes)) + (j - 1) * .16, rates,
                    yerr=np.maximum(0, [rates - low, high - rates]),
                    fmt='o', capsize=3, label=method)
    ax.set(xticks=range(len(scenes)), xticklabels=scenes, ylim=(-.05, 1.05),
           ylabel='Success fraction (Wilson 95% CI)')
    ax.legend()
    ax.grid(alpha=.25)
    fig.savefig(output / 'success_rates.png', dpi=160)
    plt.close(fig)
    fig, axes = plt.subplots(3, 1, figsize=(11, 10), layout='constrained')
    for ax, metric in zip(axes, METRICS[2:]):
        for j, (label, _) in enumerate(COMPARISONS):
            group = [r for r in tests if r['comparison'] == label and r['metric'] == metric and
                     r['median_delta'] is not None]
            x = [scenes.index(r['scenario']) + (j - .5) * .15 for r in group]
            y = np.array([r['median_delta'] for r in group])
            ax.errorbar(x, y, yerr=np.maximum(0, [y - [r['ci_low'] for r in group],
                                                  [r['ci_high'] for r in group] - y]),
                        fmt='o', capsize=3, label=label)
        ax.axhline(0, color='grey', linewidth=1)
        ax.set(xticks=range(len(scenes)), xticklabels=scenes,
               ylabel=f'{metric}\nmedian C2 - comparator (95% CI)')
        ax.legend()
        ax.grid(alpha=.25)
    fig.savefig(output / 'differences.png', dpi=160)
    plt.close(fig)
    timing = []
    for row in rows:
        log = Path(row['output']) / 'navigation.log'
        if not log.is_file():
            continue
        text = log.read_text()
        rates = re.findall(r'Control loop missed.*?Current loop rate is ([0-9.]+) Hz', text)
        cycles = float(row['duration_s']) * 10
        warnings = text.count('Control loop missed')
        timing.append({
            'scenario': row['scenario'],
            'method': row['method'],
            'seed': row['seed'],
            'real_time_factor': row['real_time_factor'],
            'warning_count': warnings,
            'estimated_miss_percent': 100 * warnings / cycles if cycles else None,
            'lowest_warned_hz': min(map(float, rates)) if rates else None,
        })
    if timing:
        write_csv(output / 'control_timing.csv', timing)
        aggregates = []
        for method in METHODS:
            group = [r for r in timing if r['method'] == method]
            rtfs = [float(r['real_time_factor']) for r in group if r['real_time_factor']]
            aggregates.append({
                'method': method,
                'n': len(group),
                'rtf_median': float(np.median(rtfs)) if rtfs else None,
                'rtf_min': min(rtfs) if rtfs else None,
                'warnings': sum((r['warning_count'] for r in group)),
                'warning_trials': sum((r['warning_count'] > 0 for r in group)),
            })
        write_csv(output / 'timing_summary.csv', aggregates)
        (output / 'timing.md').write_text(table(aggregates) + '\n\n'
                                          '周期割合の分母は10Hz×走行秒の概算。\n')


def main():
    """Compare one input CSV and optional explicit retry index."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('results', type=Path)
    parser.add_argument('--replacements', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rows = load_results(args.results, args.replacements)
    summary, tests = analyze(rows)
    write_outputs(rows, summary, tests, args.output)
    print(f'{len(rows)} trials; outputs: {args.output}')


if __name__ == '__main__':
    main()
