#!/usr/bin/env python3
"""Direction-preserving manual intent and final simulator collision guard."""
from __future__ import annotations
from collections import deque
import math
from collections.abc import Callable, Sequence

Vector3 = tuple[float, float, float]


class TimedVelocitySample:
    """Bind one received command to its own freshness time and sequence.

    ``snapshot`` is either None or an immutable (vector, receipt_wall, sequence)
    tuple. Invalid receptions invalidate the previous command; selecting a
    command never changes its receipt time or extends its lifetime.
    """

    def __init__(self) -> None:
        self.sequence = 0
        self.snapshot: tuple[Vector3, float, int] | None = None

    def receive(self, vector: Sequence[float], receipt_wall: float) -> None:
        self.sequence += 1
        self.snapshot = None
        try:
            if len(vector) != 3:
                return
            values = tuple(float(component) for component in vector)
            receipt = float(receipt_wall)
        except (TypeError, ValueError, OverflowError):
            return
        if not math.isfinite(receipt) or not all(math.isfinite(v) for v in values):
            return
        self.snapshot = (values, receipt, self.sequence)

    def select(self, now_wall: float, active: bool,
               timeout_s: float = .10) -> tuple[Vector3, int, float]:
        sample = self.snapshot
        if sample is None:
            return (0.0, 0.0, 0.0), 0, math.inf
        vector, receipt, sequence = sample
        try:
            now = float(now_wall)
        except (TypeError, ValueError, OverflowError):
            return (0.0, 0.0, 0.0), 0, math.inf
        if not math.isfinite(now):
            return (0.0, 0.0, 0.0), 0, math.inf
        age = now - receipt
        try:
            timeout = float(timeout_s)
        except (TypeError, ValueError, OverflowError):
            return (0.0, 0.0, 0.0), 0, age
        if (not active or not math.isfinite(age) or not math.isfinite(timeout)
                or timeout < 0.0 or age < 0.0 or now > receipt + timeout):
            return (0.0, 0.0, 0.0), 0, age
        return vector, sequence, age


def desired_velocity_from_direction(forward: float, left: float, vertical: float,
                                    max_speed: float, max_z_speed: float) -> Vector3:
    """Encode one 3-D input direction and magnitude, without axis-wise speeds."""
    norm = math.sqrt(forward * forward + left * left + vertical * vertical)
    if norm <= 1.0e-9:
        return (0.0, 0.0, 0.0)
    scale = max_speed / max(1.0, norm)
    if abs(vertical) * scale > max_z_speed:
        scale = max_z_speed / abs(vertical)
    return (scale * forward, scale * left, scale * vertical)


def vector_safe_command(position: Sequence[float], command: Sequence[float],
                        dt: float, collides: Callable[[Vector3], bool]
                        ) -> tuple[Vector3, bool]:
    """Accept the entire planned vector or stop it; never choose another axis."""
    vector = tuple(float(value) for value in command)
    candidate = tuple(float(position[i]) + vector[i] * dt for i in range(3))
    if not all(math.isfinite(value) for value in vector) or collides(candidate):
        return (0.0, 0.0, 0.0), True
    return vector, False


def velocity_response(velocity: Sequence[float], desired: Sequence[float],
                      dt: float, tau: float, acceleration_limit: float) -> Vector3:
    """Isotropic plant: first-order motion and bounded constant-deceleration stop."""
    error = tuple(float(desired[i]) - float(velocity[i]) for i in range(3))
    length = math.sqrt(sum(value * value for value in error))
    braking = math.sqrt(sum(float(v)*float(v) for v in desired)) < 1e-8
    scale = 1.0 if braking else min(1.0, dt / max(tau, 1e-6))
    if length > 1e-12:
        scale = min(scale, acceleration_limit * dt / length)
    return tuple(float(velocity[i]) + scale * error[i] for i in range(3))


class BoundedTimingSamples:
    """8192 recent intervals; all-run mean/max/count, bounded resident storage."""
    def __init__(self):
        self.values = deque(maxlen=8192)
        self.count = 0
        self.total = 0.0
        self.maximum = 0.0

    def append(self, value):
        if math.isfinite(value) and value >= 0.0:
            self.values.append(value)
            self.count += 1
            self.total += value
            self.maximum = max(self.maximum, value)

    def summary(self):
        if not self.count:
            return 0.0, 0.0, 0.0, 0.0
        ordered = sorted(self.values)
        return (self.total / self.count,
                ordered[round(.5 * (len(ordered) - 1))],
                ordered[round(.95 * (len(ordered) - 1))], self.maximum)
