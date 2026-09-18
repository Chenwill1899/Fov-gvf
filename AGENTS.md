## 用户项目指令

我的指令不一定对，在回答或者执行我期望的指令时，先理性分析。

## 分支工作日志

- 在 `feature/local-modifications` 分支修改源码、配置、脚本或文档时，必须同步更新仓库根目录的 `WORK_LOG.md`。
- 每条记录至少写明日期、修改目的、涉及文件、具体内容和验证结果。
- 不得把尚未执行的测试写成已通过；失败或未验证的结果也要如实记录。

<!-- hiagent:research-flywheel:start -->
# Research Flywheel Rules

These rules apply when using the HiAgent research workflow. Ordinary project questions and unrelated changes keep their existing scope. Use the project-local controller under .research-system/tools/research_flywheel.py. The main conversation acts as Governor and delegates bounded specialist work when useful; do not start nine agents for every task.

1. User instructions and recorded human decisions override Agent suggestions.
2. Read .research/graph.json and the assigned node before acting.
3. Work only within the assigned read and write scope.
4. Workers never edit .research/graph.json; emit an outcome envelope instead.
5. Do not invent papers, citations, measurements, files, tests, approvals, or completed work.
6. Preserve raw data, terminal runs, failed experiments, and contradictory findings.
7. Execution status, scientific verdict, and freshness are separate. A completed task may still refute a hypothesis. Apply the node acceptance checks.
8. When upstream inputs change, treat dependent artifacts as stale until revalidated.
9. Stop only at a verified node outcome, an external blocker, or a recorded human gate.
10. Keep REFUTED, INSUFFICIENT_EVIDENCE, and INVALID_RUN distinct.
11. Claim a task before execution, retain its attempt ID and input snapshot, and submit outcomes through the controller. Route alone does not reserve work.
12. Pass the requested model explicitly to the runtime; native role files do not override models. Mark actual model use unverified unless supported by host/provider metadata.
13. Read .research/project.json and the relevant recent attempts when resuming. Follow existing user authorization; do not ask them to relay packets between agents.
<!-- hiagent:research-flywheel:end -->
