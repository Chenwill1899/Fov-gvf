---
name: research-graph-protocol
description: Govern graph-based research work, including planning, routing, handoffs, resumption, checkpoints, evidence state, stale propagation, and human gates. Use for multi-stage paper and experiment workflows; do not use for a standalone one-step task.
---

# Research Graph Protocol

Treat the research graph as the authoritative project state. Conversation memory and Agent summaries are advisory until backed by graph nodes or artifacts.

## Concepts

- An Agent defines role and authority.
- A Skill defines a reusable procedure.
- A graph node defines one concrete unit of work and its acceptance checks.
- An artifact is an inspectable file produced by work.

## Rules

1. Read the assigned node, direct dependencies, required artifacts, write scope, and acceptance checks before acting.
2. Workers never edit .research/graph.json. Write artifacts and one outcome envelope to .research/inbox instead.
3. Only the Research Governor validates and applies outcomes.
4. Do not treat a model statement or successful command as evidence of completion.
5. Preserve failed, refuted, insufficient, invalid, partial, blocked, and stale outcomes distinctly.
6. Never overwrite terminal Run artifacts or raw evidence. Create a new version or superseding node.
7. When an upstream artifact changes, mark affected descendants stale until independently revalidated.
8. Record human decisions only after actual user input. Never synthesize approval.
9. Stop at a verified outcome, a concrete external blocker, or a human gate.

## Handoff

Every worker outcome identifies the Agent, node, attempt, status, artifacts with hashes, checks with evidence, observed facts, blockers, next actions, model, and reasoning effort.

Version 2 separates execution status, scientific verdict, and freshness. Only an independently audited verdict can support a scientific assertion; completed drafting is still not_assessed. Claim a node before execution, retain the input snapshot and attempt ID, and recheck them when applying the outcome. Relevant prior attempts and exact claim scope belong in the handoff. Native role files do not fix a model; the Governor requests an available model explicitly and records host/provider telemetry if available. Worker self-report does not verify model use.
