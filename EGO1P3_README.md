# EGO1P3 连续360°深度避障

2026-10-06从EGO1P2完整复制建立，EGO1P2原2922文件保留。2026-10-07已完成v22与P2的24轮冻结对照：
**连续扫向明显改善，全面优胜验收未通过**。扫向误差30.68→7.53°、停车.717→.250s；独立旋转部分指标退化，
两版固定目标均5/5安全到达，但P3平均70.143s略慢于P2的69.717s。CTest22/22、24轮独立扫掠和限速契约通过。
完整退化、证据与复现见[360°实现与验收](OMNI_ACCEPTANCE.md)、[当前版本](EGO1P3_VERSION.md)和[工作日志](WORK_LOG.md)。

默认主链：四路深度采集位姿 → world球面方向融合 → 跨相机候选统一选择 →
单一三维矢量整形 → 原完整运动/制动认证。四相机原始安全证据与未知拒绝保留。
联合观测覆盖证明细分深度5→7，1024预算及证据包含条件保持。水平360°方向域不等于无传感器盲区；物理相机、机体和动力学未改。

历史项目状态原样保留于[EGO1P2来源说明](EGO1P2_VERSION.md)及各旧验收报告。
它们记录的是原版本成绩，不是EGO1P3新成绩。

可用`FOV_GVF_OMNI_DEPTH=0`加主运行命令做旧候选前端对照；默认值1使用新融合。
固定目标模式启用融合时也消费四路，关闭融合时保留原前向+历史策略。

## 构建与运行

```bash
cd /home/starry/isaac-data/EGO1P3
bash scripts/build_isaac_ros_workspace.sh
bash scripts/run_isaac_fov_gvf_navigation.sh
```

需要本机 `/opt/ros/humble`、`/home/starry/isaac-data/isaacsim`、可用 NVIDIA 图形环境及北通 A2P3A BFM/XInput 手柄。默认打开 4 倍 Cloud、Isaac Sim 和 RViz；手柄四轴回中 0.5 秒后解锁。关闭窗口或 Ctrl-C 退出。

无手柄时显式使用键盘：

```bash
ISAAC_MANUAL_INPUT_MODE=keyboard bash scripts/run_isaac_fov_gvf_navigation.sh
```

键盘 W/S、A/D、R/F 为世界坐标前后、左右、升降；Space 停止，1/3/V 切换视角。平地可用 `FOV_GVF_SCENE_MODE=flat` 检查启动和渲染，但当前几何初始化只认证默认Cloud，平地不会自动放行运动；关闭 RViz 用 `FOV_GVF_RVIZ=false`，限时用 `ISAAC_MANUAL_TIMEOUT=5`。

## 保留目录

| 路径 | 用途 |
|---|---|
| scripts/*.sh | 仅两个主入口：构建、运行 |
| scripts/isaac/ | Isaac 运行器、手柄输入、三维方向输入与整体碰撞停止 |
| scripts/lib/ | BFM/XInput 设备识别 |
| src/pc_gvf/ | C++ 避障、launch、测试及 Python 数值/合成验收基准 |
| src/pc_gvf_msgs/ | PositionCommand 消息 |
| src/pc_gvf_platforms/ | 命令桥、RViz/在线地图和相关测试工具 |
| scenes/ | Cloud USD+occupancy.bin、平地导航 USD |
| tools/ | 算法基线生成与 ROS 回归探针，不是日常运行入口 |
| performance/ | 逐次运行性能记录（含继承历史） |

构建产物使用 `/tmp/fov_gvf_ego1p3_isaac_{build,install,log}`，无需项目内 build/install/log。修改源码后重新构建。算法参数集中于 `src/pc_gvf/launch/isaac_cloud_navigation.launch.py`。

当前人工输入只提供一个三维期望方向和幅值，规划器在方向图上求解单一三维建议矢量；全向方向图使用统一world坐标，后端局部方向场可随实测速度转动，几何证据始终来自四路真实相机。总速度上限2m/s、垂直分量上限1m/s均通过整体缩放实现，没有独立升降或XYZ速度叠加。方向图、参考延续、制动检查及失效停车均位于同一链路。

仿真执行模型统一使用tau=0.22s的一阶运动响应，零指令按1.2m/s²制动到停止，与预测模型对应；这是执行模型，不是第二个方向控制器。深度、位姿、人工意图或指令过期时整体停车；外部ESDF若介入也整体停止，不提取轴向滑动。

四路水平相机不是球面覆盖。纯上下输入可在已认证起始邻域或观测体积内得到三维规划结果，不能进入未观测的上/下空间。设置 `FOV_GVF_VERIFIED_START=0` 可关闭起始几何认证，近场包络无法认证时应停车。当前证据保留参数针对静态Cloud；换场景/传感器需重新核对初始化与深度契约。

ROS域42和单实例锁仍与其他副本共用，不要同时运行。
当前版本见 [EGO1P3_VERSION.md](EGO1P3_VERSION.md)，来源版本和整理归档见 [EGO1P1_VERSION.md](EGO1P1_VERSION.md)，实际变更与验证见 [WORK_LOG.md](WORK_LOG.md)。


严格论文模式回归（合成深度验证停车契约，非飞行到达验收）：

```bash
source /opt/ros/humble/setup.bash
source /tmp/fov_gvf_ego1p3_isaac_install/setup.bash
ROS_DOMAIN_ID=83 PYTHONNOUSERSITE=1 /usr/bin/python3 tools/ros_vector_guidance_probe.py --omni
```

`--omni`检查新融合前端的未知拒绝及缺失前相机；改为`--paper`检查旧深度前端，省略选项可回归保留的旧控制器。探针无起始自由邻域初始化，不能把其停车契约通过当成导航到达；完整运动证据请看验收报告。

无手柄重复已保存的三维输入/扰动轨迹：

```bash
ISAAC_MANUAL_INPUT_MODE=trace ISAAC_HEADLESS=1 FOV_GVF_RVIZ=false \
ISAAC_INTENT_TRACE="$PWD/src/pc_gvf/test/fixtures/paper/isaac_intent_trace.json" \
ISAAC_MANUAL_TIMEOUT=12 bash scripts/run_isaac_fov_gvf_navigation.sh
```

固定向前输入的Cloud绕障复现（1m/s、29秒）：

```bash
ISAAC_MANUAL_INPUT_MODE=trace ISAAC_HEADLESS=1 FOV_GVF_RVIZ=false \
ISAAC_INTENT_TRACE="$PWD/src/pc_gvf/test/fixtures/paper/isaac_verified_motion_trace.json" \
ISAAC_MANUAL_TIMEOUT=29 bash scripts/run_isaac_fov_gvf_navigation.sh
```

2026-09-24旧参数最终实测前进17.519m、外部碰撞保护介入0；复杂运行实际约43–47Hz，不能称为稳定50Hz或完整论文复现。完整验收与失败记录见 [PAPER_ACCEPTANCE.md](PAPER_ACCEPTANCE.md)。


2026-09-28默认包络半径为0.58m，其中实际USD机体（含螺旋桨）外接半径按0.48m计算。
运动、当前位置及到达记录使用同一最小认证范围；松手制动后的实测位置也会保存。
0.10m为期望前视下限；默认联合选择更短前视和低速，最短尝试仍受实测速率制动需求及5mm计算保留量约束。所有输出继续经过完整制动检查。参数依据与对照见最新验收报告。
诊断日志 `PAPER_EVIDENCE` 给出包络半径、最大认证前缀、保留/撤销球及过期/淘汰关键帧计数。

复现包络改向测试并导出独立碰撞分析：

```bash
ISAAC_MANUAL_INPUT_MODE=trace ISAAC_HEADLESS=1 FOV_GVF_RVIZ=false \
ISAAC_INTENT_TRACE="$PWD/src/pc_gvf/test/fixtures/paper/isaac_envelope_recovery_trace.json" \
ISAAC_ACCEPTANCE_TRACE=/tmp/ego1p3-envelope.csv ISAAC_MANUAL_TIMEOUT=94 \
bash scripts/run_isaac_fov_gvf_navigation.sh
PYTHONNOUSERSITE=1 /usr/bin/python3 tools/analyze_envelope_trace.py /tmp/ego1p3-envelope.csv
```

对照余量可用 `FOV_GVF_SAFETY_MARGIN`（默认0.02m）、`FOV_GVF_ROLLOUT_MARGIN`（默认0.02m）；
`FOV_GVF_MIN_LOOKAHEAD` 默认0.10m（期望下限），`FOV_GVF_ADAPTIVE_LOOKAHEAD=0` 可用于关闭自适应前视的对照实验。`FOV_GVF_BODY_RADIUS` 同时传给规划与仿真外部保护，
默认0.48m，不能为了通行而缩到实际机体以内。额外余量只在当前理想静态仿真中校核，未标定实物。


## 停车诊断与离线重放

`PAPER_BUILD`区分前视阈值未认证、当前方向不在自由连通域、源/目标不可分离、粗场或细场退化等。
`best_prefix`为已证明的建域阈值下界，不能当作真实障碍距离。`ISAAC_ACCEPTANCE_TRACE`现在同时记录
构场原因、所需前缀、通过的方向数、通道数和是否使用加密图。

默认在`/tmp/fov_gvf_ego1p3_replays/<run_id>/`保存最近16组`.bin`状态与`.json`原决策；连续拒绝
后最多每5秒保存一次，环形覆盖该次运行最旧的诊断帧。可设`FOV_GVF_REPLAY_DIR=/绝对路径`保留到
指定目录，设为空字符串禁用。快照在全部传感器证据接入后、当前step之前采集，包括配置、证据、
方向场和参考状态；重放比较控制决策，不复现GPU渲染或墙钟超时。

```bash
/usr/bin/python3 tools/verify_navigation_replays.py /tmp/fov_gvf_ego1p3_replays/<run_id>
/tmp/fov_gvf_ego1p3_isaac_install/pc_gvf/lib/pc_gvf/paper_replay /绝对路径/frame_0.bin
```

使用对应版本的可执行程序验证原决策；换算法后的重放可用于对照，但不应再要求结果逐值一致。
快照为本机标量二进制格式，拒绝截断/超长数据；不是跨架构ROS bag。原始全分辨率障碍点已经在
捕获前用于撤销证据，单步重放使用撤销后的状态与保守深度，不重新执行传感器回调。
