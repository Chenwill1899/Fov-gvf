"""Protocol regressions: failure and braking must never masquerade as arrival."""
import json
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'scripts' / 'isaac'))
from navigation_benchmark import GoalTrial


@pytest.fixture
def trial():
    return GoalTrial(json.loads((ROOT / 'performance/navigation_benchmark_20260929/protocol.json').read_text()))


def test_same_direction_and_limits_without_map_access(trial):
    assert trial.intent((-64., -11.2, 2.8), 1.9) == (0., 0., 0.)
    for position in ((-64., -11.2, 2.8), (64., 11.2, -10.), (62., 12., 6.)):
        q = trial.intent(position, 2.)
        delta = tuple(a-b for a, b in zip(trial.goal, position))
        scales = [u/d for u, d in zip(q, delta) if abs(d) > 1e-12]
        assert scales == pytest.approx([scales[0]]*len(scales))
        assert math.hypot(*q) <= 2.+1e-12 and abs(q[2]) <= 1.+1e-12


def test_near_goal_is_not_arrived_until_slow_and_held(trial):
    g = trial.goal
    assert trial.intent(g, 3.) == (0., 0., 0.)
    assert trial.update(g, (1., 0., 0.), 3., 100., False) == 'RUNNING'
    assert trial.update(g, (0., 0., 0.), 4., 101., False) == 'RUNNING'
    assert trial.update(g, (0., 0., 0.), 4.4, 101.4, False) == 'RUNNING'
    assert trial.update(g, (0., 0., 0.), 4.5, 101.5, False) == 'ARRIVED'
    result = trial.result('ARRIVED', g, 4.5, 101.5, 0)
    assert result['simulation_elapsed_s'] == 2.5
    assert result['wall_elapsed_s'] == 1.5


def test_moving_or_leaving_goal_restarts_hold(trial):
    g = trial.goal
    trial.update(g, (0., 0., 0.), 3., 100., False)
    assert trial.update(g, (.2, 0., 0.), 3.4, 100.4, False) == 'RUNNING'
    assert trial.update(g, (0., 0., 0.), 3.8, 100.8, False) == 'RUNNING'
    assert trial.update(g, (0., 0., 0.), 4.2, 101.2, False) == 'RUNNING'
    assert trial.update(g, (0., 0., 0.), 4.31, 101.31, False) == 'ARRIVED'


def test_collision_and_timeout_are_failures_even_at_goal(trial):
    assert trial.update(trial.goal, (0., 0., 0.), 3., 100., True) == 'COLLISION_GUARD'
    assert trial.update((-64., -11.2, 2.8), (0., 0., 0.), 242., 200., False) == 'TIMEOUT'


def test_exploration_abort_is_explicit_failure_and_disabled_by_default(trial):
    p = (-64., -11.2, 2.8)
    trial.update(p, (0., 0., 0.), 2., 100., False)
    assert trial.update(p, (0., 0., 0.), 100., 198., False) == 'RUNNING'
    exploratory = GoalTrial(trial.protocol, exploration_stall_s=30.)
    exploratory.update(p, (0., 0., 0.), 2., 100., False)
    assert exploratory.update(p, (0., 0., 0.), 32., 130., False) == 'ABORTED_EXPLORATION_STALL'
    assert exploratory.result('ABORTED_EXPLORATION_STALL', p, 32., 130., 0)['exploration_stall_s'] == 30.
