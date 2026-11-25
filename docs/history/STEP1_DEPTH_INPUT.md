> 历史Python版本记录（17:40）。当前实现已全部迁移到C++，默认本机5070/GPU物理、0.5s验证，见[当前说明](CPP_DEPTH_INPUT.md)。下文数值保留原日期含义。

# 第一步：四路同步深度与机体状态读取

本步代码已经实现，并在原Cloud40m场景运行了5.000s实际ROS仿真时钟。四路深度、标定、采集时间和机体状态读取正常；设置1ms墙钟轮询，但严格的每次1ms实时期限尚未达到。全景拼接、时间补偿、方向选择与运动控制留待后续步骤。

## 代码和运行

- 核心算法输入文件：`src/new_controller/new_controller/depth_input.py`。
- ROS读取与逐次计时：`src/new_controller/new_controller/depth_reader_node.py`。
- 固定机体的实际Isaac传感器测试：`scripts/isaac/run_depth_input_simulation.py`。
- 独占锁、单发布者预检、启动/退出与源码冻结：`scripts/run_depth_step1.py`、`scripts/run_depth_step1.sh`。

```bash
cd /home/starry/isaac-data/EGO4P0
bash scripts/build_isaac_ros_workspace.sh
bash scripts/run_depth_step1.sh
```

默认5s从第一组完整输入就绪后的`/clock`起算，不包含Isaac启动和数据等待。保留原320×240、60Hz名义仿真步长、原相机位置/朝向和Cloud场景。专用测试入口按每次update至少1/60s墙钟节拍推进，慢帧不追赶加速；实测渲染/传输/系统调度仍决定墙钟时长。节点和Isaac结束后退出，无后台续跑。原`run_manual.sh`仍要求未来真正具有运动输出的控制器，不能将本读取节点当成飞行控制器。

可选`--scene /绝对路径/scene.usd`、`--duration 5`、`--output /尚不存在的目录`；输出不覆盖旧实验。使用ROS域42、localhost、原共用`/tmp/fov_gvf_user_navigation.lock`，预检控制/相机/时钟发布者为0；本步不创建PositionCommand、速度或意图发布者。

## 输出与约定

`DepthInput.read()`返回`(ControllerInput, camera_read_ms)`；输入未齐返回`(None, [])`，格式或标定错误明确抛错。由一个ROS executor线程持有接收缓存和读取，后续跨线程使用需另外同步。

| 对象 | 输出 | 约定 |
|---|---|---|
| 四路图像 | `value.cameras[0..3].D`，对应D1～D4 | 前、左、后、右；float32、240×320、米；每次实际复制全部像素，返回只读自有数组 |
| 内参 | `.K` | 3×3，从ROS CameraInfo读取并与当前USD实际投影核对 |
| 外参 | `.T_BC` | 4×4；`p_B=T_BC*p_C`；启动时读实际USD相对机体变换，静态只读保存 |
| 时间 | `.t_ns` | 原始图像采集时间，整数纳秒；重复读取保持原时间，不改写为轮询时间 |
| 深度类型 | `.depth_type` | `optical_axis_z_m`，光轴Z；不当成欧氏射线距离 |
| 机体速度 | `value.state.v_B` | 当前机体系速度m/s；本Isaac raw世界系速度按`R_WB.T @ v_W`转换 |
| 机体姿态 | `value.state.R_WB` | 3×3，将机体向量旋转到世界系；由里程计四元数读取 |
| 状态时间 | `value.state.t_ns` | 状态原时间，单独记录与当前clock的差；不假定等于图像时间 |

光学系为右/下/前，机体系为前/左/上。当前四路实际K相同：

```text
K = [[160, 0, 160], [0, 160, 120], [0, 0, 1]]
```

光心分别约(0.22,0,0.02)、(0,0.22,0.02)、(-0.22,0,0.02)、(0,-0.22,0.02)m，完整旋转矩阵保存在`sensors.json`和NPZ的TBC中；不以名义角度替代实际USD读回。

四路按相同整数采集时间组包，最多保留8个未完成时间组；迟到/重复或被新完整组替代的部分组均计数。只接受完整四路，较新单路到达不会和旧路混成一组。没有新完整组时重复读取最近完整组并标记`new_frame=0`。这是1ms读取最新输入，不是1kHz新图像采集。不对四路或机体状态做时间补偿。

原始NaN、Inf、非正值保留，统计时仅有限正数作为有效深度。Inf不被替换成“已证明自由空间”。检查字节序、行stride、消息长度、帧名、分辨率、显式深度类型、K、畸变及位姿合法性。

## 最终5秒实测

证据目录：[depth_step1_20261101_173735_586663](../../../EGO4P0/docs/performance/depth_step1/depth_step1_20261101_173735_586663)。本次读取功能检查通过；严格1ms期限未通过，不能用平均值替代最坏值。

| 指标 | 实测 |
|---|---:|
| 仿真读取窗口 | 5.000s，clock 0.133 → 5.133s |
| 对应墙钟读取窗口 | 5.456916s |
| 实际完整读取次数 | 5450 |
| 新同步组 / 重复读取 | 296 / 5154 |
| 四路收到消息数（含重复时间戳） | [301, 300, 299, 299] |
| 未完成组被丢弃 / 迟到或重复消息 | 4 / 9 |
| 完整组最大四路时间差 | 0ns |
| 完整读取耗时均值 / P95 / P99 / 最大 | 0.137888 / 0.201111 / 0.257840 / 1.108812ms |
| 读取耗时超过1ms | 1次 |
| 实际轮询间隔均值 / P99 / 最大 | 1.001453 / 1.316758 / 2.567156ms |
| 实际间隔超过1.5ms | 24次 |
| 有效读取频率（墙钟） | 998.549Hz |
| 新同步组频率（图像仿真时间） | 59.197Hz |
| 深度年龄均值 / 最大 | 19.425 / 50.000ms仿真时间 |
| 状态年龄最大 | 16.667ms仿真时间 |

`read_ms`包含四张完整像素复制、内参/元数据校验、状态读取与世界→机体速度转换；各相机读取另分项。它不包含渲染、DDS接收反序列化、额外逐帧像素统计或结束时CSV/NPZ写盘。`tick_including_statistics_ms`另记录计时行/像素统计处理；调度间隔以单调时钟独立记录。本测试是Python ROS2普通定时器，不能作为硬实时保证或完整控制算法耗时。

每个新同步组四路均有有限正深度。最后一组有效像素/Inf为：前75964/836、左73284/3516、后70898/5902、右76044/756；每路共76800像素，NaN与非正值均0。Inf原样保留。机体固定在(-64,-11.2,2.8)m，v_B=0、R_WB=I符合本测试条件。源场景SHA不变，模拟器与读取进程均退出0。没有飞行/避障能力结论。

## 数据和验证

- [`reader_summary.json`](../../../EGO4P0/docs/performance/depth_step1/depth_step1_20261101_173735_586663/reader_summary.json)：全部统计、实际标定与计时边界。
- [`reads.csv`](../../../EGO4P0/docs/performance/depth_step1/depth_step1_20261101_173735_586663/reads.csv)：每次真实读取的耗时/间隔/四路采集时间/新帧标识/数据年龄。
- [`frames.csv`](../../../EGO4P0/docs/performance/depth_step1/depth_step1_20261101_173735_586663/frames.csv)：每个新同步组四路像素有效性和距离范围。
- [`first_input.npz`](../../../EGO4P0/docs/performance/depth_step1/depth_step1_20261101_173735_586663/first_input.npz)、[`last_input.npz`](../../../EGO4P0/docs/performance/depth_step1/depth_step1_20261101_173735_586663/last_input.npz)：完整D1～D4、K1～K4、TBC1～TBC4、t1～t4、深度类型、v_B、R_WB和状态时间。
- [`manifest.json`](../../../EGO4P0/docs/performance/depth_step1/depth_step1_20261101_173735_586663/manifest.json)、`source_at_run.tar.gz`：源码/实际安装模块身份、场景路径和运行ID。
- [`validation.json`](../../../EGO4P0/docs/performance/depth_step1/depth_step1_20261101_173735_586663/validation.json)、`verify_saved_output.py`：独立从保存数组和CSV复算计数/时间/标定/像素账/源文件身份；通过。

4个ROS包构建成功；新增6项输入检查通过，7个Python文件AST与Shell语法通过。未重跑原231项基础回归（其代码未改），未执行导航/实机。两次前置5秒实测都保留：17:34不加墙钟节拍、17:35全局绝对节拍；最终17:37逐帧最小周期避免暖机后追赶，用最终数据报告，不选取前两轮较小最大值冒充最终结果。

后续若要求每次严格不超过1ms，需要进一步处理实时调度与读取/诊断执行架构，并重新实测；第一步暂不扩展相机帧率、全景拼接或控制输出。
