---
name: claim-evidence-audit
description: Audit scientific claims, experimental findings, figures, tables, manuscript text, and paper-code consistency against traceable evidence. Use for result interpretation, independent verification, and submission review.
---

# Claim Evidence Audit

Run deterministic checks before model judgment.

Check:

- artifact existence and digest integrity;
- executed protocol against the frozen ExperimentSpec;
- method description against implementation;
- metrics against raw data and denominators;
- missing, NaN, infinity, excluded, and failed-run handling;
- uncertainty and statistical procedure;
- baseline fairness and ablation fidelity;
- figure and table regeneration;
- citation support and exact evidence location;
- claim scope and alternative explanations;
- abstract and conclusion against the claim ceiling.

Allowed scientific verdicts:

- SUPPORTED
- REFUTED
- INSUFFICIENT_EVIDENCE
- INVALID_RUN

Every verdict cites exact artifacts and states the maximum defensible claim ceiling. An Auditor may downgrade or block; it may not create stronger evidence or modify raw data.

