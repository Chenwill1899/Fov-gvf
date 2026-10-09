#!/usr/bin/env python3
"""Compare intent alignment and command continuity in recorded Isaac traces.

Commands use matching diagnostic input (not asynchronous joystick samples).
Measured reverse travel is reported both raw and excluding two seconds after
an input turn >=45 degrees, during which physical braking is expected.
"""
import argparse
import csv
import json
import math
from pathlib import Path


def vec(row, names):
    return tuple(float(row.get(k) or 0) for k in names)


def norm(v):
    return math.sqrt(sum(x*x for x in v))


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def subtract(a, b):
    return tuple(x-y for x, y in zip(a, b))


def percentile(values, fraction):
    if not values:
        return None
    values = sorted(values)
    return values[min(len(values)-1, int(fraction*(len(values)-1)))]


def analyze(path):
    rows = list(csv.DictReader(path.open()))
    reverse_time = reverse_streak = reverse_longest = stable_reverse = 0.
    intended_progress = active_distance = 0.
    previous_q = (0., 0., 0.)
    last_turn = -100.
    stamps = set()
    commands = []
    acceleration = []
    acceleration_vectors = []
    jerk = []
    for i, row in enumerate(rows):
        t = float(row['t'])
        dt = float(rows[i+1]['t'])-t if i+1 < len(rows) else 0
        q, v = vec(row, ('qx', 'qy', 'qz')), vec(row, ('vx', 'vy', 'vz'))
        if norm(q) > .05 and norm(previous_q) > .05:
            if dot(q, previous_q)/(norm(q)*norm(previous_q)) < math.cos(math.pi/4):
                last_turn = t
        previous_q = q
        if norm(q) > .05:
            intended_progress += dt*dot(v, q)/norm(q)
            active_distance += dt*norm(v)
        reverse = norm(q) > .05 and norm(v) > .05 and dot(q, v) < -1e-6
        if reverse:
            reverse_time += dt
            reverse_streak += dt
            reverse_longest = max(reverse_longest, reverse_streak)
            if t-last_turn >= 2:
                stable_reverse += dt
        else:
            reverse_streak = 0.
        stamp = row.get('diagnostic_stamp')
        if stamp and stamp not in stamps:
            stamps.add(stamp)
            command = vec(row, ('command_x', 'command_y', 'command_z'))
            intent = vec(row, ('input_x', 'input_y', 'input_z'))
            commands.append((float(stamp), command, intent))
        if i > 0:
            dt_measured = t-float(rows[i-1]['t'])
            previous_v = vec(rows[i-1], ('vx', 'vy', 'vz'))
            if dt_measured > 0:
                a = tuple(x/dt_measured for x in subtract(v, previous_v))
                acceleration.append(norm(a))
                if acceleration_vectors:
                    jerk.append(norm(subtract(a, acceleration_vectors[-1]))/dt_measured)
                acceleration_vectors.append(a)
    steps = [norm(subtract(b[1], a[1])) for a, b in zip(commands, commands[1:])]
    moving_commands = [r for r in commands if norm(r[1]) > .001 and norm(r[2]) > .05]
    negative = [r for r in moving_commands if dot(r[1], r[2]) < -1e-6]
    return {'trace': str(path), 'diagnostic_samples': len(commands),
            'moving_command_samples': len(moving_commands),
            'negative_intent_command_samples': len(negative),
            'intended_direction_progress_m': intended_progress,
            'active_distance_m': active_distance,
            'direction_progress_per_distance': intended_progress/max(1e-12, active_distance),
            'measured_reverse_seconds': reverse_time,
            'longest_measured_reverse_seconds': reverse_longest,
            'measured_reverse_outside_turn_braking_2s': stable_reverse,
            'command_vector_step_max_mps': max(steps, default=0.),
            'command_vector_step_p95_mps': percentile(steps, .95),
            'command_vector_step_rms_mps': math.sqrt(sum(s*s for s in steps)/max(1, len(steps))),
            'command_steps_over_025_mps': sum(s > .25 for s in steps),
            'measured_acceleration_max_mps2': max(acceleration, default=0.),
            'measured_jerk_rms_mps3': math.sqrt(sum(j*j for j in jerk)/max(1, len(jerk))),
            'measured_jerk_p95_mps3': percentile(jerk, .95),
            'measured_jerk_max_mps3': max(jerk, default=0.)}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('traces', type=Path, nargs='+')
    args = parser.parse_args()
    print(json.dumps([analyze(p) for p in args.traces], indent=2))
