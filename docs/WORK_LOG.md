# EGO4P5 工作日志

## 2026-11-25 · GitHub 发布分支 EGO4P5

本次发布以远端 EGO2P6 的末次提交 `2954c36f1d983ca6f93f08a298e828b42a76db4c` 为父提交，发布内容为本目录整理后的双场景版本 `572cf3fde07ef841a26e9a2b`。源码、场景、测试夹具、本地依赖和冻结控制器一并入库。上传前在此目录运行 R05 与四门机库的 `--check` 均通过。早期实验和恢复快照的跨项目链接仍指向原 isaac-data 工作区。[GitHub 发布分支](https://github.com/Chenwill1899/Fov-gvf/tree/EGO4P5)

## 2026-11-25 · 按 EGO2P6 整理目录

根目录从 10 个业务目录收拢为 `docs/`、`scenes/`、`scripts/`、`src/`。首页与技术索引合并到 [README.md](README.md)。45 份早期文档和讲义移入 [history](history)。脚本顶层由 76 个文件降至 25 个，另分 [数据分析](../scripts/analysis)、[历史基准](../scripts/benchmarks) 与 [场景工具](../scripts/scene_tools)。历史文档中 182 处失效引用接回有对应文件或目录的来源工程，EGO2P5 的一处改为已归档原件的恢复说明。

| 原位置 | 现位置 |
| --- | --- |
| `apps/` 下的三个应用 | [双场景巡航](../src/auto_cruise)、[R05 模型](../src/r05_model)、[早期闭环](../src/isaac_closed_loop) |
| `candidates/ldlt_parallel/new_controller`、`config/` | [调和场与跟踪核心](../src/new_controller/ldlt)、[参数](../src/new_controller/config) |
| `tests/`、`third_party/` | [球面回归与输入夹具](../src/auto_cruise/tests)、[本地库](../src/third_party) |
| `libexec/auto_cruise/` | [巡航发布](../src/auto_cruise/releases) |

**发布与验证。** 从新路径重建控制器，ELF 与源发布逐字节一致。294 项输入全部对应，7 项路径文本发生变化，19 项动态依赖哈希相同。正式双场景入口及两个历史策略选项的预检通过。Release 主套件 CTest 26/26、独立监督器 1/1 通过，R05 模型也已构建。双场景原始轨迹复算得到相同的审计 JSON。四相机归档输入的深度补值和像素射线核对通过；两份配置生成检查通过，colcon 仍识别原有四个 ROS 包。[发布与核验细目](../../HISTORY/snapshots/EGO4P5_layout_20261125/verification.json)

**验证范围。** 本机缺少 Ninja，首次 CMake 配置失败，改用 Unix Makefiles 后完成全量构建和测试。旧分析脚本把 `--help` 作为运行数据目录，调用失败；另以模块方式验证导入并用真实存档测试预处理核验脚本。旧参考场景脚本以隔离 Python 启动时找不到同目录模块，以普通 Python 启动后返回原有的停用提示。未启动新的 Isaac 闭环、复测实时调度或重跑历代基准。

**恢复。** [完整快照](../../HISTORY/snapshots/EGO4P5_layout_20261125/before.tar.gz) 保存整理前的目录、文件权限和符号链接，已与原树比对。SHA-256 为 `2c8d286a7c8a9b55d37272d7b45f010e5d3e33da88b1cfc3839d54ff4b6c10fd`。在一个新的空目录解开可取回旧版 `EGO4P5/`，包括原始发布和 294 项输入。具体路径见[逐文件对照](../../HISTORY/snapshots/EGO4P5_layout_20261125/paths.json)。

## 2026-11-25 · 从指定的双场景发布建立项目

从 EGO4P4 的发布清单、同版镜像和后续修改前备份恢复默认控制器、仿真入口和构建脚本，保留当时的技术文档、ROS 对照代码、测试夹具、场景及依赖。项目只附带一个正式控制器发布。审计运行目录、录像、旧工作日志、编译与 Python 缓存均留在副本之外。

本次逐项校验 294 项发布输入和控制器 SHA-256。分别以 `--scenario r05 --check` 和 `--scenario hangar --check` 验证源码、依赖和场景，二者均选择默认方向策略。在临时目录构建 Release 测试目标并运行 CTest，26/26 通过。本次未运行 Isaac Sim 飞行。[算法状态和归档数据出处](PROJECT_STATUS.md)
