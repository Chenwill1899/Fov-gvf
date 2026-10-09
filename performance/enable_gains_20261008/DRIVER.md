# 功能重新启用的有限对照

驱动：`tools/run_enabled_features_ablation.py`。本准备阶段仅 AST、配置集合及 `--help` 检查通过，未启动 Isaac 或构建。实际运行和结果由主任务登记。

`--case goal|jitter`；`--profiles` 默认 baseline、shared、memory、dynamic、uncertainty、combined；默认1轮，后续偶数轮逆序。baseline六关，单项仅开对应功能，combined开①②④⑥；③⑤始终关，执行源callback。使用当前安装的默认goal/manual程序，不设置可执行覆盖；批次开始冻结二进制、源码、安装launch、场景、输入和分析器SHA。

结果写入本目录 `closed_loop/LABEL`，拒绝覆盖。主Shell负责Isaac生命周期与串行锁。每轮独立全机体扫掠审计、全时段运动限值检查及实际启动路径/SHA核验；固定目标额外核验实际生效六开关。默认手动runtime只直接记录程序/launch身份，其开关由显式环境与冻结launch绑定，不声称有额外参数回读。

目标TIMEOUT保存为 `safe_arrival=false`、`audit_safe=false`；仅当正常进程退出、几何/运动硬门及身份完整通过才继续下一配置。`manifest.complete=true`表示计划全部执行，不等于目标全部到达；超时总数单列。碰撞、外部保护、运动限值、缺结果或身份失败停止且保留此前全部结果。

示例（未在准备阶段执行）：

```bash
python3 tools/run_enabled_features_ablation.py p5_enabled_goal_20261008 --case goal
python3 tools/run_enabled_features_ablation.py p5_enabled_jitter_20261008 --case jitter
```
