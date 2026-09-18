# Python behavior fixture format

These fixtures freeze the output of the Python depth-angular algorithm before
its behavior-equivalent C++ migration. They are migration data, not a claim that
every frozen edge-case behavior is safe.

Generate them from the repository root with:

```bash
PYTHONNOUSERSITE=1 /usr/bin/python3 tools/generate_python_baseline.py --force
```

Each `*.fixture` file uses UTF-8 `key=value` lines. Arrays are row-major,
comma-separated decimal values. `nan`, `inf`, `-inf`, and `none` are explicit.
The format intentionally needs no JSON or NumPy reader so the C++ parity test can
consume it with the standard library.

Timing is excluded because it is not deterministic. The frozen outputs are the
depth input, backprojected obstacle points, direction free distance, planning
mask, safe-goal selection, connected-component labels, harmonic potential,
discrete path and angular limiting, reference correction, braking and rollout
decisions, source/goal/command pixels, field validity, and final core velocity.
