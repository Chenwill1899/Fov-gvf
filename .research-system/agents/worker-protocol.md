# Worker protocol v2

Read the full assigned packet and role before acting: question, node version, attempt ID, input/dependency snapshot, relevant prior attempts, acceptance criteria, outputs, skills, scope, budget and expiry. A task needs an active claim lease. Report missing context instead of inventing it.

Use the minimum relevant artifacts, including prior failures. Keep scientific questions separate from command success. A non-Run task returns partial when progress is useful but unfinished. An active Run keeps its lease and records progress instead; its final outcome is completed, failed or blocked. Use completed only with all declared outputs and acceptance checks; each check cites a hashed artifact. Scientific verdicts belong to the Evidence Auditor, not the code writer or runner.

Do not edit graph.json, reassign models, consume extra resources beyond the task, overwrite raw runs, or submit a guessed runtime model. Preserve process references when a job is still active. Write the v2 outcome under .research/inbox/ using the assigned attempt ID. The Governor applies it after independent checks.

For an experiment iteration, record what changed, why it can distinguish competing explanations, what was learned, and the next smallest useful test. Repeated failures without new information require diagnosis rather than unbounded retries. Draft prose remains within the exact audited evidence and scope.
