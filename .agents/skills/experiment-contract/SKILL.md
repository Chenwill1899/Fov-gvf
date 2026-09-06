---
name: experiment-contract
description: Design, freeze, review, and execute scientific experiment protocols with explicit comparisons, metrics, validity criteria, budgets, and evidence ceilings. Use for pilots, confirmatory studies, baselines, and ablations.
---

# Experiment Contract

Freeze the primary protocol before observing the result it will judge.

Every ExperimentSpec defines:

- linked hypothesis and claim;
- comparison arms and fair-comparison conditions;
- dataset, scenario, and independent experimental unit;
- primary and secondary metrics with units and direction;
- baselines and ablations;
- seeds, repetitions, or deterministic coverage rationale;
- success, falsification, invalid-run, and stopping conditions;
- resource budget and timeout;
- raw artifacts that must be retained;
- analysis and uncertainty procedure;
- maximum evidence ceiling if successful;
- version and frozen status.

Do not silently edit a frozen specification. Create a new version and preserve the old one. Pilot evidence remains pilot-level until a separately frozen confirmatory protocol passes. A Runner executes the contract; an independent Auditor decides what the evidence supports.

