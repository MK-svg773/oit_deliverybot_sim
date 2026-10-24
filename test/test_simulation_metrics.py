import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from simulation_metrics import clearance_to_circle, TrialMetrics, support_contact, interpolate_truth


def test_floor_support_requires_normals_and_rejects_wall_or_body_contacts():
    assert support_contact('deliverybot::left_wheel::wheel school_mesh::surface', [-0.001], [1.0])
    assert not support_contact('deliverybot::left_wheel::wheel school_mesh::surface', [-0.001], [])
    assert not support_contact('deliverybot::left_wheel::wheel school_mesh::surface', [0.09], [])
    assert not support_contact('deliverybot::base_link::body school_mesh::surface', [0.0], [])
    assert not support_contact('deliverybot::left_wheel::wheel school_mesh::surface', [0.0], [0.0])


def test_clearance_truth_is_interpolated_and_never_extrapolates_stale_data():
    history=[(10.0,0.0,0.0),(10.1,1.0,2.0)]
    assert interpolate_truth(history,10.05)==pytest.approx((0.5,1.0))
    with pytest.raises(ValueError):interpolate_truth(history,10.2)
    with pytest.raises(ValueError):interpolate_truth([(9.,0.,0.),(11.,1.,1.)],10.)


def test_duration_includes_start_and_terminal_event_instead_of_sampling_window():
    m=TrialMetrics(start_time=0.)
    m.add(.1,0.,0.,0.,None)
    m.add(1.1,1.,0.,.2,None)
    assert m.result('success',terminal_time=1.25)['arrival_s']==1.25


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
