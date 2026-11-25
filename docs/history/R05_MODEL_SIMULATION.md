本项目主入口和当前验证见 [EGO4P5 项目说明](../README.md)。下文保留 R05 的模型、引导和上游实验背景。

2026-11-18T20:58:47+08:00 — **R05 跟踪优先策略已正式复飞。** 使用 ARL-Robot-1 无人机外观，五线程 FIFO5 闭环 31.655 仿真秒到达并停稳，原版 41.500 s；独立扫掠零重叠，机体净空下界 0.386 m。低速引导仍处理原视频急弯。 [验证报告](../../../EGO4P3/docs/performance/r05_cap_check_20261118/REPORT.md) · [本轮正常倍速视频](../../../EGO4P3/docs/performance/r05_visual_match_20261118/replay_20261118_205608_200128/R05_EGO4P3_ARL_reference_view.mp4)。

2026-11-18T20:45:53+08:00 — **当前R05显示已改为P6 ARL新外观及原视频风格。** 新回放1600×900/30fps/43.533s，实时与回放共用呈现模块。[命令与外观/物理范围](R05_PRESENTATION.md) · [验证报告](../../../EGO4P3/docs/performance/r05_visual_match_20261118/REPORT.md)。旧蓝色简模与1280×720/25fps条目仅作历史。

# R05 原模型在 EGO4P3 中的仿真入口

本入口把此前R05 Isaac飞行的模型和实验接线同步到P3。Cloud USD、占据文件、无人机外观、原四相机安装及历史路线来自原R05实验；当前C++核心直接编译自P3根目录的 `candidates/ldlt_parallel/new_controller`。不读取P2的安装或冻结ELF，也不依赖P2/P6路径运行。

这是“原R05实验接线＋当前P3核心”的独立仿真测试。原实验含路线引导、备用搜索/加减速及历史净空策略；该入口不代表当前P3生产滚动控制器的纯终点自主导航能力。现有 `run_isaac_closed_loop.py` 的四个无预设路线测试保持原入口。

## 按实时状态和深度选择起转时机

**当前 Isaac Sim 自动巡航主版本为二维深度图优化版 B。** 默认构建和启动使用五轮 FIFO5 对照中的 B 版控制器及配套仿真协议，保留逐拍扫掠与制动净空检查。版本 C 的执行回执、故障制动接管与停稳重入暂不启用。 [本次默认入口双场复飞](../../../EGO4P3/docs/performance/image_depth_main_20261120/verified_flights.json) · [主版本构建身份](../../../EGO4P3/docs/performance/image_depth_main_20261120/build_identity.json) · [三版同期对照](../../../EGO4P3/docs/performance/three_versions_fifo_20261120/matrix_20261120_154257_611687/REPORT.md)。

当前版本用物理相机的二维图像与光轴深度计算可见目标的起转偏差、球体可见性和疑似障碍的像素复核。起步、终点以及视野外的方向仍能用真实位姿处理。两场共用下方入口。[二维计算与 FIFO5 对照](../../../EGO4P3/docs/performance/image_depth_20261119/REPORT.md)

在项目普通用户的终端启动。默认打开 Isaac Sim，使用 ARL-Robot-1 外观、四个深度相机和五线程 FIFO5。无窗口复测可在命令末尾附加 `--headless`。运行目录由程序打印。

```bash
/usr/bin/python3 -I /home/starry/isaac-data/EGO4P4/scripts/run_auto_cruise.py --scenario r05
/usr/bin/python3 -I /home/starry/isaac-data/EGO4P4/scripts/run_auto_cruise.py --scenario hangar
```

两场共用一个控制器，几何路线提供局部意图。R05 使用原路线，机库路线穿过四个门并在高门后停稳。速度按实测三维速度向量的模统计。启动时检查控制器、动态库、源文件、场景和占据图的身份；运行中读回算法线程及传感器时钟。重建命令为 `/usr/bin/python3 -I EGO4P4/scripts/build_auto_cruise.py`，在工作区根目录执行。可用启动命令加 `--check` 单独核对输入。[两场复飞与完整 FIFO5 决策计时](../../../EGO4P3/docs/performance/auto_turn_20261119/REPORT.md)

## 复现航点 204 提前转向的恒速试验

要复现 R05 Cloud 的 12 m/s 恒速试验，以普通用户运行下面的命令。入口会先核对固定构建身份，再为本次子进程设置 FIFO5 所需的优先级限额，启动无窗口仿真，并用逐拍轨迹和独立碰撞审计进行核验。需要时终端会提示输入 sudo 密码。

```bash
/usr/bin/python3 -I /home/starry/isaac-data/EGO4P3/scripts/run_r05_cruise.py
```

同一命令加 `--check` 只核验构建；加 `--verify <本次试验运行目录>` 可对已有 FIFO5 日志复核。标准输出分别列出接管前滚动三段、接管后完整决策和全部运动拍决策的均值、最近秩 P99、最大值及超过 5 ms 的次数。巡航速度按三维实际速率和原实验的起步、终点空间边界核对。此入口采用该场景预选的接管位置。通用 R05 引导仿真的命令见下节。

## 启动命令

R05 现采用跟踪优先的速度策略。实际速度已接近 12 m/s 时，只要当前参考、下一步和制动路径通过检查，P3 就可以继续执行恒速跟踪。进入低速引导或终点制动时，原路线的速度建议继续生效。窗口中的 ARL-Robot-1 是无人机外观，控制及扫掠检查沿用 R05 的机体半径和动力学。[限速对照、局部观测与复飞记录](../../../EGO4P3/docs/performance/r05_cap_check_20261118/REPORT.md)

已构建后，启动可视化仿真并要求五线程FIFO5：

```bash
sudo /usr/bin/python3 -I /home/starry/isaac-data/EGO4P3/scripts/run_r05_model.py
```

默认：显示窗口，5ms物理步长，0.02倍最低墙钟节拍，最多110仿真秒，开启原R05几何路线引导。0.02倍下长距离飞行需要较长墙钟时间。仅加快仿真测试节拍可用 `--time-scale 1`，物理步长仍为5ms，实际运行速度还受渲染计算制约。`--headless`用于无窗口；`--seconds 1`用于有界短测，短测结束不代表到达终点。

只读检查：

```bash
/usr/bin/python3 -I /home/starry/isaac-data/EGO4P3/scripts/run_r05_model.py --check
```

P3核心或入口修改后重新构建：

```bash
/usr/bin/python3 -I /home/starry/isaac-data/EGO4P3/scripts/build_r05_model.py
```

显式普通调度功能检查：同一运行命令加 `--correctness-only` 并去掉sudo。这类结果不得标为FIFO耗时。正式入口权限不足或线程读回不符即停止，不自动降级。编译器、Isaac与监督进程不整体升为FIFO，只有五个算法线程设置FIFO5/CPU0～4。

## 已同步内容

- `scenes/r05_cloud/ego_swarm_cloud_navigation.usd`：历史20m高的R05 Cloud自包含场景，包含原机器人与四相机；不是P2后来40m高的活动Cloud。
- `scenes/r05_cloud/occupancy.bin`：1200×300×50、0.4m分辨率，供出生点证书与独立扫掠碰撞检查。
- `scenes/r05_cloud/r05_guidance.csv`：原R05存档；运行时仅读取xyz几何引导，不回放其速度、位姿或执行输入。`--goal-only`可关闭路线，但不保证无引导时能到达。
- `apps/r05_model/`：原实验控制/审计逻辑、当前SDK时钟适配及独立CMake。
- `apps/r05_model/src/verified_start.cpp`及两个registry、`include/pc_gvf/verified_start.hpp`：原本依赖EGO2P6的初始净空认证代码现已本地保存。
- `scripts/build_r05_model.py`、`scripts/run_r05_model.py`：独立构建/身份检查/权限控制/线程核对/结果和审计入口。

模型与占据文件原SHA分别为 `cfcc22694241eb2ca5e506060c136a2f9fe5b3b5e2efd76cf0d44a1da26ed1ca` 和 `e4e7e4301ae0ba6ae129bf813285ceda6d5f625f80a00b65284368682d9e68b8`。启动前与同步时均核对。未复制原实验视频、完整日志或缓存；新测试的缓存与结果在P3自己的performance目录中生成。

## 模型与时间一致性

执行器沿用一阶速度响应：τ=0.10s、最大速度12m/s、加速度校验150m/s²、半径0.10m。Isaac接收每步控制后更新模型位置和速度，再渲染下一组真实四路320×240光轴深度。这是运动学闭环，不是四旋翼推力/电机/姿态刚体动力学。

原图无命中按原R05的Isaac专用开关处理：核实相机远裁面后将+Inf映射到4.5m，导航证据范围仍是4m；其他无效像素不因此授予净空。原相机安装几何保持；标定采用RTX像素中心对应的整数索引主点159.5/119.5。

当前SDK使用暂停时间线、每拍显式推进一个真实5ms物理步、时间线同步到读回物理时间、零时间渲染。四路传感器时间必须与实际物理时间一致；初始化预热单独处理。GUI显示和墙钟节拍不改变积分dt。原脚本首步退出及预热停滞的尝试保留在本次报告中。

## 输出及验收边界

每次新建 `docs/performance/r05_model_sync_20261117/run_*` 并打印RESULT_DIR。保存源码/ELF/动态库身份、模型SHA、五线程读回、场景/相机读取、原图、控制/航迹、物理和四路传感器时间、执行结果及独立扫掠审计。窗口关闭/异常日志、未到达和失败结果均保留。

`status=TIMEOUT`表示该次仿真窗口结束，不能当作到达成功；`ARRIVED`才进一步核对终点及保持状态。是否完整复飞成功、FIFO是否实际生效以及当前性能，须以对应新运行的execution/result/audit为准，不能沿用P2的41.50s和旧耗时。生产安装和默认运动关闭配置保持，运动只在此独立仿真进程中执行。

最新本次验证见 [同步与验证报告](../../../EGO4P3/docs/performance/r05_model_sync_20261117/REPORT.md)。

## 2026-11-18 全程 FIFO5 与正常倍速视频

本次完整引导飞行在五线程 FIFO5 下到达终点。飞行 41.50 仿真秒、308.925 m，终点误差 0.169 m，停稳保持 0.50 s。独立扫掠复核为零重叠，最小净空下界 0.560 m。8,200 次运动控制中 552 次使用滚动参考分支，7,648 次使用 R05 引导分支。 [本次运行](../../../EGO4P3/docs/performance/r05_model_sync_20261117/run_20261118_195738_748686/simulation/result.json) · [独立审计](../../../EGO4P3/docs/performance/r05_model_sync_20261117/run_20261118_195738_748686/simulation/audit.json)

[打开 1× 视频](../../../EGO4P3/docs/performance/r05_video_20261118/normal_speed/R05_Cloud_EGO4P3_1x.mp4)。影片在原 Cloud USD 中按本次记录的三维位姿和偏航角跟拍，每 8 个 5 ms 物理步取一帧。H.264、1280×720、25 fps，1,038 帧共 41.52 s，最后一帧按固定帧率显示。画面下方显示对应仿真时间、速度和高度。
