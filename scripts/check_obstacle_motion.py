#!/usr/bin/env python3
"""Numerically check complete Gazebo schedules, using recorded samples only."""
import argparse
import csv
import json
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from simulation_scenarios import resolve_scenario
import yaml


def check_batch(root):
    """Check measured motion independently against recorded resolved schedules."""
    config = yaml.safe_load((Path(get_package_share_directory('oit_deliverybot_sim')) /
                             'config/simulation/scenarios.yaml').read_text())
    checks = []
    for result in csv.DictReader((root / 'results.csv').open()):
        rows = list(csv.DictReader((Path(result['output']) / 'samples.csv').open()))
        environment = yaml.safe_load(Path(result['environment']).read_text())
        obstacle = resolve_scenario(config, result['scenario'], int(result['seed']),
                                    origin=environment.get('scenario_origin'))
        segments = json.loads(result['actual_segments'])
        delay = float(result['actual_delay_s']) if obstacle else 0.
        boundaries = []
        if obstacle:
            end = delay
            boundaries.append(end)
            for segment in segments:
                end += segment['duration']
                boundaries.append(end)
            complete = bool(rows) and float(rows[-1]['time_s']) >= end + .5
            position_errors = []
            for row in rows:
                t = float(row['time_s'])
                segment_start, displacement = delay, 0.
                for segment in segments:
                    active = max(0., min(segment['duration'], t - segment_start))
                    displacement += segment['speed'] * active
                    segment_start += segment['duration']
                ex = obstacle['x'] + math.cos(obstacle['yaw']) * displacement
                ey = obstacle['y'] + math.sin(obstacle['yaw']) * displacement
                position_errors.append(math.hypot(float(row['obstacle_x_m']) - ex,
                                                  float(row['obstacle_y_m']) - ey))
            max_position = max(position_errors, default=math.inf)
            errors = []
            for a, b in zip(rows, rows[1:]):
                ta, tb = float(a['time_s']), float(b['time_s'])
                if any(ta - .15 <= t <= tb + .15 for t in boundaries):
                    continue
                vx = (float(b['obstacle_x_m']) - float(a['obstacle_x_m'])) / (tb - ta)
                vy = (float(b['obstacle_y_m']) - float(a['obstacle_y_m'])) / (tb - ta)
                midpoint, speed, segment_start = (ta + tb) / 2, 0., delay
                for segment in segments:
                    if segment_start <= midpoint < segment_start + segment['duration']:
                        speed = segment['speed']
                    segment_start += segment['duration']
                errors.append(math.hypot(vx - speed * math.cos(obstacle['yaw']),
                                         vy - speed * math.sin(obstacle['yaw'])))
            max_velocity = max(errors, default=math.inf)
        else:
            complete, max_position, max_velocity = bool(rows), 0., 0.
        # Position allowance covers four 50ms command transitions at <=0.5m/s,
        # plus 5ms physics steps. Velocity excludes 150ms around transitions.
        passed = complete and max_position <= .12 and max_velocity <= .02
        checks.append({
            'scenario': result['scenario'], 'method': result['method'], 'seed': result['seed'],
            'complete': complete, 'max_position_error_m': max_position,
            'max_velocity_error_mps': max_velocity, 'position_tolerance_m': .12,
            'velocity_tolerance_mps': .02, 'boundary_exclusion_s': .15, 'passed': passed})
    with (root / 'motion_checks.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(checks[0]))
        writer.writeheader()
        writer.writerows(checks)
    for check in checks:
        print(check)
    return all(check['passed'] for check in checks)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('batch', type=Path)
    raise SystemExit(0 if check_batch(parser.parse_args().batch) else 1)
