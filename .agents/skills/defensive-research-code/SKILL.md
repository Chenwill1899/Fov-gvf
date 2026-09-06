---
name: defensive-research-code
description: Harden experiment code, data and log parsers, artifact writers, long-running jobs, external commands, GPU work, concurrency, and physical hardware. Use at trust, data-integrity, process, and hardware boundaries; do not add defensive scaffolding to ordinary pure internal code.
---

# Defensive Research Code

Apply protection at boundaries where failure can corrupt evidence, lose data, escape scope, waste resources, or move hardware.

## Required behavior

1. Validate external inputs and critical parameters before use.
2. Keep missing, unknown, zero, NaN, and infinity distinct.
3. Never substitute stale output for missing evidence.
4. Bound retries, timeouts, output size, and resource use.
5. Use unique Run directories and immutable raw logs.
6. Write important mutable files atomically.
7. Record code, configuration, environment, seed, input, and artifact provenance with hashes.
8. Reject path escape, symlink artifacts, ambiguous targets, and accidental overwrites.
9. Preserve failure details and use structured error categories.
10. Avoid blanket exception handling and silent fallback.
11. Leave the smallest meaningful executable check for non-trivial logic.

For physical hardware, retain calibration, command limits, watchdogs, emergency stop, and explicit human authorization.

These protections override Ponytail minimalism. After they are satisfied, use Ponytail to remove all remaining unnecessary complexity.

