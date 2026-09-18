#!/usr/bin/env python3
"""Pure helpers for independent horizontal and vertical manual control."""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence


Vector3 = tuple[float, float, float]


def compose_desired_velocity(
        forward: float,
        left: float,
        vertical: float,
        max_xy_speed: float,
        max_z_speed: float) -> Vector3:
    """Normalize XY alone, then scale and clamp Z independently."""
    horizontal_norm = math.hypot(forward, left)
    if horizontal_norm > 1.0:
        forward /= horizontal_norm
        left /= horizontal_norm
    return (
        max_xy_speed * forward,
        max_xy_speed * left,
        max_z_speed * max(-1.0, min(1.0, vertical)),
    )


def componentwise_safe_command(
        position: Sequence[float],
        command: Sequence[float],
        dt: float,
        collides: Callable[[Vector3], bool]) -> tuple[Vector3, bool]:
    """Keep a safe Z command when the combined or horizontal move is blocked.

    This is a final simulator-side emergency guard using the authoritative ESDF
    or scene bounds.  It is deliberately not a 3-D path planner.
    """
    full = tuple(float(position[i]) + float(command[i]) * dt for i in range(3))
    command_tuple = tuple(float(command[i]) for i in range(3))
    if not collides(full):
        return command_tuple, False

    vertical_command = (0.0, 0.0, command_tuple[2])
    vertical_candidate = (
        float(position[0]),
        float(position[1]),
        float(position[2]) + command_tuple[2] * dt,
    )
    if abs(command_tuple[2]) > 1.0e-9 and not collides(vertical_candidate):
        return vertical_command, True

    horizontal_command = (command_tuple[0], command_tuple[1], 0.0)
    horizontal_candidate = (
        float(position[0]) + command_tuple[0] * dt,
        float(position[1]) + command_tuple[1] * dt,
        float(position[2]),
    )
    if math.hypot(*horizontal_command[:2]) > 1.0e-9 and not collides(horizontal_candidate):
        return horizontal_command, True
    return (0.0, 0.0, 0.0), True
