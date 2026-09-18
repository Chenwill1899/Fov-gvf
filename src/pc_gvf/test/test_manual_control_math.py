from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest


ISAAC_SCRIPT_DIR = Path(__file__).resolve().parents[3] / "scripts" / "isaac"
sys.path.insert(0, str(ISAAC_SCRIPT_DIR))

from manual_control_math import (  # noqa: E402
    componentwise_safe_command,
    compose_desired_velocity,
)


MAX_XY = 2.0
MAX_Z = 1.0


def velocity(forward=0.0, left=0.0, vertical=0.0):
    return compose_desired_velocity(forward, left, vertical, MAX_XY, MAX_Z)


def test_required_fifteen_manual_input_cases():
    cases = {
        "W": (1, 0, 0),
        "S": (-1, 0, 0),
        "A": (0, 1, 0),
        "D": (0, -1, 0),
        "W+A": (1, 1, 0),
        "W+D": (1, -1, 0),
        "S+A": (-1, 1, 0),
        "S+D": (-1, -1, 0),
        "UP": (0, 0, 1),
        "DOWN": (0, 0, -1),
        "W+UP": (1, 0, 1),
        "A+UP": (0, 1, 1),
        "S+DOWN": (-1, 0, -1),
        "W+D+UP": (1, -1, 1),
        "RELEASED": (0, 0, 0),
    }
    for name, axes in cases.items():
        actual = velocity(*axes)
        horizontal_requested = math.hypot(axes[0], axes[1]) > 0.0
        if horizontal_requested:
            assert math.hypot(actual[0], actual[1]) == pytest.approx(MAX_XY), name
        else:
            assert actual[:2] == (0.0, 0.0), name
        assert actual[2] == pytest.approx(axes[2] * MAX_Z), name


def test_xy_normalization_does_not_include_z():
    diagonal = velocity(1, -1, 1)
    assert math.hypot(diagonal[0], diagonal[1]) == pytest.approx(MAX_XY)
    assert diagonal[2] == MAX_Z


def test_collision_guard_preserves_safe_vertical_component():
    def horizontal_wall(candidate):
        return candidate[0] > 0.05

    safe, blocked = componentwise_safe_command(
        (0.0, 0.0, 1.0), (1.0, 0.0, 0.5), 0.1, horizontal_wall)
    assert blocked
    assert safe == (0.0, 0.0, 0.5)


def test_collision_guard_stops_unsafe_vertical_component():
    def low_ceiling(candidate):
        return candidate[2] > 1.02

    safe, blocked = componentwise_safe_command(
        (0.0, 0.0, 1.0), (0.0, 0.0, 0.5), 0.1, low_ceiling)
    assert blocked
    assert safe == (0.0, 0.0, 0.0)
