from __future__ import annotations
import math
import sys
from pathlib import Path
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "scripts" / "isaac"))
from manual_control_math import (TimedVelocitySample, desired_velocity_from_direction,
                                 vector_safe_command)


@pytest.mark.parametrize("axes", [(1,0,0),(-1,0,0),(0,1,0),(0,-1,0),
    (0,0,1),(0,0,-1),(1,1,1),(1,-1,-1),(0.2,0.1,0.05)])
def test_manual_input_preserves_one_3d_direction(axes):
    velocity = desired_velocity_from_direction(*axes, 2.0, 1.0)
    scale = next(v / a for a, v in zip(axes, velocity) if a)
    assert velocity == pytest.approx(tuple(scale * a for a in axes))
    assert math.sqrt(sum(v*v for v in velocity)) <= 2.0 + 1e-12
    assert abs(velocity[2]) <= 1.0 + 1e-12


def test_released_input_stops():
    assert desired_velocity_from_direction(0,0,0,2,1) == (0,0,0)


@pytest.mark.parametrize("collides", [lambda p: p[0] > 0.05, lambda p: p[2] > 1.02])
def test_guard_stops_whole_vector_instead_of_sliding(collides):
    assert vector_safe_command((0,0,1),(1,0,0.5),0.1,collides) == ((0,0,0), True)


def test_guard_preserves_clear_vector():
    assert vector_safe_command((0,0,1),(1,-0.3,0.5),0.1,lambda p: False) == ((1,-0.3,0.5), False)


def test_validation_plant_obeys_vector_acceleration_limit():
    from manual_control_math import velocity_response
    actual = velocity_response((0,0,0), (1,1,1), 0.02, 0.22, 1.2)
    assert math.sqrt(sum(v*v for v in actual)) == pytest.approx(0.024)
    assert actual[0] == actual[1] == actual[2]


def test_validation_plant_does_not_overshoot_or_stop_instantly():
    from manual_control_math import velocity_response
    assert velocity_response((0,0,0), (0.01,0,0), 1.0, 0.22, 1.2) == (0.01,0,0)
    assert 0.0 < velocity_response((1,0,0), (0,0,0), 0.02, 0.22, 1.2)[0] < 1.0


def test_zero_command_braking_matches_certified_distance_and_stops():
    from manual_control_math import velocity_response
    velocity=(1.,0.,0.); distance=0.
    for _ in range(60):
        velocity=velocity_response(velocity,(0,0,0),.02,.22,1.2)
        distance+=.02*velocity[0]
    assert velocity == (0.,0.,0.)
    assert distance <= 1./(2*1.2)



def test_timed_sample_binds_immutable_vector_to_its_receipt():
    sample = TimedVelocitySample()
    assert sample.sequence == 0 and sample.snapshot is None
    vector = [1., -.3, .5]
    sample.receive(vector, 10.)
    old_snapshot = sample.snapshot
    vector[0] = 99.
    assert old_snapshot == ((1., -.3, .5), 10., 1)
    sample.receive((-.2, .4, 0.), 10.08)
    assert old_snapshot == ((1., -.3, .5), 10., 1)
    assert sample.snapshot == ((-.2, .4, 0.), 10.08, 2)
    selected, sequence, age = sample.select(10.09, True)
    assert selected == (-.2, .4, 0.) and sequence == 2
    assert age == pytest.approx(.01)


def test_timed_sample_repeats_without_refreshing_receipt_or_sequence():
    sample = TimedVelocitySample()
    sample.receive((.8, .2, 0.), 10.)
    assert sample.select(10.02, True)[:2] == ((.8, .2, 0.), 1)
    assert sample.select(10.09, True)[:2] == ((.8, .2, 0.), 1)
    assert sample.snapshot == ((.8, .2, 0.), 10., 1)
    assert sample.select(10.10, True)[:2] == ((.8, .2, 0.), 1)
    assert sample.select(10.101, True)[:2] == ((0., 0., 0.), 0)
    assert sample.select(10.101, True)[2] == pytest.approx(.101)


def test_timed_sample_release_is_immediate_even_when_latest_command_is_fresh():
    sample = TimedVelocitySample()
    sample.receive((1., 0., .2), 5.)
    assert sample.select(5., False) == ((0., 0., 0.), 0, 0.)
    assert sample.select(5.05, False)[:2] == ((0., 0., 0.), 0)
    assert sample.snapshot == ((1., 0., .2), 5., 1)


def test_timed_sample_missing_or_clock_rollback_fails_closed():
    sample = TimedVelocitySample()
    selected, sequence, age = sample.select(5., True)
    assert selected == (0., 0., 0.) and sequence == 0 and math.isinf(age)
    sample.receive((1., 0., 0.), 5.)
    assert sample.select(4.9, True)[:2] == ((0., 0., 0.), 0)
    assert sample.select(4.9, True)[2] == pytest.approx(-.1)


@pytest.mark.parametrize("vector,receipt", [
    ((1., 2.), 5.), ((1., 2., 3., 4.), 5.),
    ((math.nan, 0., 0.), 5.), ((1., math.inf, 0.), 5.),
    ((1., 0., -math.inf), 5.), ((1., 0., 0.), math.nan),
    ((1., 0., 0.), math.inf), ((1., 0., 0.), -math.inf),
    ((1., "bad", 0.), 5.), (None, 5.), ((1., 0., 0.), None),
])
def test_invalid_receipt_clears_previous_command_and_next_valid_sample_recovers(vector, receipt):
    sample = TimedVelocitySample()
    sample.receive((.8, 0., 0.), 4.98)
    sample.receive(vector, receipt)
    assert sample.sequence == 2 and sample.snapshot is None
    assert sample.select(5., True)[:2] == ((0., 0., 0.), 0)
    sample.receive((-.3, .1, .2), 5.)
    assert sample.select(5.01, True)[:2] == ((-.3, .1, .2), 3)


@pytest.mark.parametrize("now,timeout", [
    (math.nan, .1), (math.inf, .1), (-math.inf, .1), (None, .1),
    (5.05, math.nan), (5.05, math.inf), (5.05, -.1), (5.05, None),
])
def test_invalid_selection_times_fail_closed(now, timeout):
    sample = TimedVelocitySample()
    sample.receive((.8, 0., 0.), 5.)
    assert sample.select(now, True, timeout)[:2] == ((0., 0., 0.), 0)
    assert sample.snapshot == ((.8, 0., 0.), 5., 1)
