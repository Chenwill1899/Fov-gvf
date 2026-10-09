# EGO1P5 开发要求

- EGO1P5 是用户指定的新开发副本，2026-10-07完整复制自EGO1P3当前v22状态；EGO1P3原目录保留，不自动同步或修改。EGO1P2是更早基线，EGO1P4是独立项目。
- 用户指令需要先理性分析，再执行。
- 开始修改前阅读 /home/starry/isaac-data/HISTORY/README.md 和 RECORDING_RULES.md。
- 修改源码、配置、脚本、场景、文档或运行方式时，同步更新本项目 WORK_LOG.md 与 HISTORY/CHANGELOG.md。
- 测试未执行不得写成通过，失败与限制必须记录。
- 日常构建和运行仅使用 scripts 下两个主 Shell 入口；运行路径与归档见 README.md 和 EGO1P5_VERSION.md。
- 研究代理框架已移入整理前快照；恢复它是独立任务，当前不启动旧研究计划。
- 继承实验与验收仍属于原EGO1P1/EGO1P2/EGO1P3；新实验使用独立EGO1P5标签，不得重命名旧成绩或覆盖失败。performance中的历史冻结/汇总脚本保留原版语义，使用前核对其版本和路径。
- 本次复制不代表算法再次优化或全面优胜；当前已知旋转退化、目标耗时/计算波动与传感器盲区见EGO1P5_VERSION.md。
