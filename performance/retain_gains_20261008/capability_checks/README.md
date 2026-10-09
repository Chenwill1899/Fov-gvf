# Frozen-library capability checks

Existing unchanged CMake test objects were relinked using each frozen core library. Each test source and repository header dependency matched all four source manifests before linking; object/library hashes, flags and original/replaced link commands are recorded in summary.json. No source was compiled, no production executable was replaced, and no Isaac or performance benchmark was run.

| Frozen library | Checks passed |
|---|---:|
| baseline | 5/5 |
| v2 | 5/5 |
| v4 | 5/5 |
| combined | 5/5 |

These are five named capability checks per library, not four complete CTest suites or a performance claim. OMP_NUM_THREADS=1 and OMP_WAIT_POLICY=PASSIVE were used consistently. Per-test outputs and binary hashes are retained.
