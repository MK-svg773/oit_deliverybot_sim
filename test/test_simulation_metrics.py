import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
# Import the source scripts after adding their directory to the module path.
from simulation_metrics import (  # noqa: E402
    clearance_to_circle, interpolate_truth, support_contact, TrialMetrics)


def test_floor_support_requires_normals_and_rejects_wall_or_body_contacts():
    assert support_contact('deliverybot::left_wheel::wheel school_mesh::surface', [-0.001], [1.0])
    assert not support_contact('deliverybot::left_wheel::wheel school_mesh::surface', [-0.001], [])
    assert not support_contact('deliverybot::left_wheel::wheel school_mesh::surface', [0.09], [])
    assert not support_contact('deliverybot::base_link::body school_mesh::surface', [0.0], [])
    assert not support_contact('deliverybot::left_wheel::wheel school_mesh::surface', [0.0], [0.0])


def test_clearance_truth_is_interpolated_and_never_extrapolates_stale_data():
    history = [(10.0, 0.0, 0.0), (10.1, 1.0, 2.0)]
    assert interpolate_truth(history, 10.05) == pytest.approx((0.5, 1.0))
    with pytest.raises(ValueError):
        interpolate_truth(history, 10.2)
    with pytest.raises(ValueError):
        interpolate_truth([(9., 0., 0.), (11., 1., 1.)], 10.)


def test_duration_includes_start_and_terminal_event_instead_of_sampling_window():
    m = TrialMetrics(start_time=0.)
    m.add(.1, 0., 0., 0., None)
    m.add(1.1, 1., 0., .2, None)
    assert m.result('success', terminal_time=1.25)['arrival_s'] == 1.25


def test_clearance_uses_asymmetric_rotated_footprint():
    assert clearance_to_circle(0, 0, 0, 1, 0, 0.2) == pytest.approx(0.69)
    assert clearance_to_circle(0, 0, math.pi, 1, 0, 0.2) == pytest.approx(0.45)
    assert clearance_to_circle(0, 0, math.pi / 2, 0, 1, 0.2) == pytest.approx(0.69)
    assert clearance_to_circle(0, 0, 0, 0, 0, 0.2) == 0


def test_metrics_use_sim_time_and_keep_failure_distinct_from_arrival():
    m = TrialMetrics()
    m.add(10, 0, 0, 0, 1.0)
    m.add(11, 1, 0, 0.2, 0.5)
    m.add(12, 1, 0, 0, 0.25)
    result = m.result('collision')
    assert result['duration_s'] == 2
    assert result['distance_m'] == 1
    assert result['stopped_s'] == 1
    assert result['min_clearance_m'] == 0.25
    assert result['arrival_s'] is None


def test_metrics_reject_time_reversal_and_nonfinite_positions():
    m = TrialMetrics()
    m.add(10, 0, 0, 0, None)
    with pytest.raises(ValueError):
        m.add(9, 0, 0, 0, None)
    with pytest.raises(ValueError):
        m.add(11, float('nan'), 0, 0, None)


def crossing_samples(speeds=None, count=21):
    """Supply known truth: the circle touches the inflated lane at .3, exits at .4."""
    return [{'time_s': i / 10, 'speed_mps': speeds.get(i, 0.) if speeds else 0.,
             'obstacle_x_m': 3., 'obstacle_y_m': [0., .2, .4, .57][i] if i < 4 else .8}
            for i in range(count)]


def score_crossing(rows, terminal=2., window=1., hold=.5):
    """Run production scoring with a straight corridor and explicit fixed settings."""
    import simulation_metrics
    assert hasattr(simulation_metrics, 'post_clearance_metrics')
    return simulation_metrics.post_clearance_metrics(
        rows, {'start': [0., 0., .04, 0.], 'goal': [6., 0., 0.],
               'obstacle_radius': .25, 'stop_speed': .03},
        {'x': 3., 'y': 0., 'yaw': math.pi / 2, 'post_clearance': {
            'margin_m': .1, 'window_s': window, 'hold_s': hold}}, terminal)


def test_exit_uses_circle_and_body_margin_then_clips_window_edges():
    result = score_crossing(crossing_samples({i: .03 for i in range(6, 21)}), window=.95)
    assert result['t_clear_s'] == .4
    assert result['post_stopped_s'] == pytest.approx(.2)
    assert result['post_observed_s'] == pytest.approx(.95)
    assert result['post_missing_s'] == pytest.approx(0.)
    assert result['post_window_complete']
    assert result['restart_category'] == 'restarted'
    assert result['restart_delay_s'] == pytest.approx(.2)


def test_restart_requires_continuous_hold_and_does_not_count_a_brief_spike():
    speeds = {5: .1, 6: .1, **{i: .03 for i in range(8, 21)}}
    result = score_crossing(crossing_samples(speeds))
    assert result['restart_delay_s'] == pytest.approx(.4)
    assert result['post_stopped_s'] == pytest.approx(.2)


def test_already_moving_is_not_zero_restart_delay():
    result = score_crossing(crossing_samples({4: .03}))
    assert result['restart_category'] == 'already_moving'
    assert result['restart_delay_s'] is None


def test_no_restart_and_early_end_are_distinct_censoring_categories():
    complete = score_crossing(crossing_samples())
    assert complete['restart_category'] == 'censored_window'
    assert complete['restart_observed_s'] == pytest.approx(1.)
    assert complete['restart_delay_s'] is None
    ended = score_crossing(crossing_samples(), terminal=.75)
    assert ended['restart_category'] == 'censored_end'
    assert ended['post_observed_s'] == pytest.approx(.3)
    assert ended['post_stopped_s'] == pytest.approx(.3)
    assert not ended['post_window_complete']


def test_hold_after_window_or_trial_end_cannot_confirm_restart():
    rows = crossing_samples({i: .03 for i in range(11, 21)})
    assert score_crossing(rows)['restart_category'] == 'censored_window'
    assert score_crossing(rows, terminal=1.5)['restart_category'] == 'censored_window'
    assert score_crossing(rows, terminal=1.3)['restart_category'] == 'censored_end'


def test_truth_gaps_are_missing_and_never_filled_as_stopped_or_restart():
    rows = crossing_samples({i: .03 for i in range(8, 21)})
    result = score_crossing([row for row in rows if row['time_s'] != .6])
    assert result['restart_category'] == 'missing_samples'
    assert result['restart_delay_s'] is None
    assert result['post_missing_s'] == pytest.approx(.2)
    assert result['post_observed_s'] == pytest.approx(.8)
    assert not result['post_window_complete']
    missing_exit = score_crossing([row for row in rows if row['time_s'] != .3])
    assert missing_exit['t_clear_s'] is None
    assert missing_exit['t_clear_status'] == 'missing_samples'


def test_failure_before_exit_is_not_assigned_scheduled_future_truth():
    result = score_crossing(crossing_samples(), terminal=.2)
    assert result['t_clear_s'] is None
    assert result['t_clear_status'] == 'not_observed'
    assert result['post_stopped_s'] is None
    assert result['restart_category'] == 'clear_not_observed'


def test_reentry_invalidates_the_clear_lane_assumption():
    rows = crossing_samples()
    rows[9]['obstacle_y_m'] = .4
    result = score_crossing(rows)
    assert result['t_clear_status'] == 'reentered'
    assert result['t_clear_s'] is None


def test_scoring_reads_csv_values_and_uses_route_normal_not_world_y(tmp_path):
    """Catch string arithmetic and a lane accidentally tied to the world y axis."""
    import csv
    from simulation_metrics import post_clearance_metrics

    rows = crossing_samples({i: .03 for i in range(6, 21)})
    for row in rows:
        row['obstacle_x_m'], row['obstacle_y_m'] = -row['obstacle_y_m'], row['obstacle_x_m']
    path = tmp_path / 'samples.csv'
    with path.open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)
    with path.open() as stream:
        result = post_clearance_metrics(
            list(csv.DictReader(stream)),
            {'start': [0., 0., .04, math.pi / 2], 'goal': [0., 6., math.pi / 2],
             'obstacle_radius': .25, 'stop_speed': .03},
            {'x': 0., 'y': 3., 'yaw': math.pi, 'post_clearance': {
                'margin_m': .1, 'window_s': 1., 'hold_s': .5}}, 2.)
    assert result['t_clear_s'] == .4
    assert result['restart_delay_s'] == pytest.approx(.2)


def test_no_observation_after_exit_and_non_crossing_are_not_complete_windows():
    """Catch goal/shutdown extrapolation and assigning zero stop to unrelated scenes."""
    from simulation_metrics import post_clearance_metrics

    result = score_crossing(crossing_samples(count=5))
    assert result['t_clear_s'] == .4
    assert result['post_observed_s'] == 0.
    assert result['restart_observed_s'] == 0.
    assert result['restart_category'] == 'censored_end'
    assert not result['post_window_complete']
    for obstacle in (None, {'x': 1.8, 'y': 0.}):
        unrelated = post_clearance_metrics([], {}, obstacle, 0.)
        assert unrelated['t_clear_status'] == 'not_applicable'
        assert unrelated['post_stopped_s'] is None


def test_exit_side_follows_motion_even_when_start_is_inside_the_lane():
    """Catch falsely reversing the exit side for a small positive initial offset."""
    from simulation_metrics import post_clearance_metrics

    rows = crossing_samples()
    rows[0]['obstacle_y_m'] = .05
    result = post_clearance_metrics(
        rows, {'start': [0., 0., .04, 0.], 'goal': [6., 0., 0.],
               'obstacle_radius': .25, 'stop_speed': .03},
        {'x': 3., 'y': .05, 'yaw': math.pi / 2, 'post_clearance': {
            'margin_m': .1, 'window_s': 1., 'hold_s': .5}}, 2.)
    assert result['t_clear_s'] == .4
