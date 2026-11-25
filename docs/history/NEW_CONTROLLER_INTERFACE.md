2026-11-04后续集成选择已固定为：**粗细两层Eigen LDLT＋严格缓存，一级4×4、二级再3×3**。候选四路摘要19200/2160；当前活动0.6接口保持，独立实现与接线边界见[PREFERRED_INTEGRATION.md](PREFERRED_INTEGRATION.md)。

2026-11-04 17:30 +0800 — **按用户要求撤回第八、九步，活动版本恢复0.6.0虚拟中心环视。** 当前仅保留步骤1～7：保留朝向/忽略平移、ρ转换、固定方向查表、0.05m分档/.10m包络、3m近块排除及候选域Ω；缺失与未覆盖仍未知。八九步模块、相关线程/话题/输出及安装残留已移出活动源码/运行目录，原数据和实验记录保留。回退后4包构建、9/9测试和一份独立离线回放通过，20份二进制输出与原0.6.0相同，18项运行身份匹配；未启动仿真。此状态优先于下方0.7.0历史说明。恢复说明见[本次回退](../../../EGO4P3/docs/ROLLBACK_STEPS8_9.md)。

2026-11-04 13:40 +0800 — **新增C++0.7.0两级场/角域曲线接口。** 当前说明见[第八、九步](STEP8_9_HARMONIC_FIELDS.md)。

- `HarmonicDepthInput.allocate/capture`：接收端从既有第一层摘要生成第二级，不扫描原图；独立SPSC深拷贝紧凑输入，无DepthFrame借用。
- `HarmonicIntent`：DesiredMotion、q0、坐标修订、R_WB/p_W/状态时间。`/controller/reference_tangent`支持base_link有限非零向量；否则v_B，静止采用期望锚向。
- `HarmonicPlanner.configure/allocate/build`：输出粗细phi、域/边界/相机位、残差/迭代/耗时、CurveSample和终止原因。生产由HarmonicWorker异步调用；输入1°，粗层3°，r=.1/Lp=3固定。
- 边界0/1/2/3/4=域外/内部/低势/出口/有效粗细耦合；节点state=0/1/2=未知/不可用/可用。不得把域外全部当未知或把Valid场当作曲线完成。
- `FineCurveIntegrator::gradient`按需返回F、gradient_B、norm2、valid、local_chart；极区F_alpha/F_beta为NaN，以三维切向量积分。Idle/exit锚点valid=false表示未求梯度。norm2对应F^TGF，零场先检查Exit，再Stagnation。
- `HarmonicWorker`有三个独立SPSC通道：ROS深度摘要→场、2ms标量→场、场→2ms完整结果。每个latest仅单消费者，持有引用直到下一次latest；不向原图缓冲新增消费者。版本不可原地改写。
- 新组重建域，确认域/边界相同后可复用解；坐标/几何/参数变化不得复用旧许可。reads.csv的field_compatible仅表示输入身份相符；执行许可、时效策略和第十六步检查仍未实现，无运动发布器。

2026-11-03 23:11 +0800 当前0.6.0按用户接受的近似统一使用机体中心虚拟环视：规划保留R、忽略t与真实近场盲区，固定查表生成ρ环视和候选域Ω；未观测/不完整仍未知。旧日期真实光心核对是历史实现，见[当前接口](STEP6_VIRTUAL_CENTER.md)。

2026-11-03 22:20 +0800 历史0.5.0新组采用块级保守方向排除，无逐近像素几何处理；余格默认未知，完整远观测只是依据，实际候选投影/原始完整性核对才可用。详见[当前C++ API及验证](STEP6_BLOCK_DOMAIN.md)。

2026-11-03 19:07 +0800 历史0.4.0接口采用紧凑观测统计与按需块/像素几何查询，详见[精简实现](STEP5_7_COMPACT_INPUTS.md)。观测数量不是求解可用节点；第八步需按真实投影建立域/一次边界，第十六步需检查反馈/限幅/响应后的实际预测运动，均尚未实现。无运动输出或新版在线成绩。

# EGO4P0 新控制器接口与待实现工作

## 当前状态

`src/new_controller/`现为独立C++17 ROS2包，已实现四路深度输入与机体状态读取，以及原始3×3异常补值、固定b/d视线表、第三步机体系二维方向来源网格和第四步深度查询，见[CPP_DEPTH_INPUT.md](DEPTH_500HZ.md)。读取节点不发布运动指令，后续控制与避障尚未实现。基础包`pc_gvf`只安装配置和接线launch，不生成旧控制器。

补值结果位于`ControllerInputView::depth->D_filled`，原始深度仍为`D`；通过`ControllerInputView::pixel_rays->at(camera,u,v)`查询b、d，初始化一次后只读复用。完整生命周期、处理规则和离线/在线验证边界见[第二步说明](STEP2_DEPTH_PREPROCESSING.md)。

通过`DepthObservationView(*input.depth)`查询像素/半开ROI最小光轴深度及覆盖计数，并按相机/块id回查代表观测；默认角格仅存数量/相机位，完整来源仅显式调试模式可枚举；结果含同组时间戳/序号，无值为std::nullopt。方向网格默认1°×1°，不把空格插值成观测、不构造自由域，也不新增V_i/U_i有效性掩码。生命周期、边界和输出格式见[第三、第四步接口](STEP3_4_DIRECTION_QUERIES.md)。

当前方向来源为3×3块代表：默认`block(camera,id)`返回相机、double中心u/v、原图半开u0/v0/u1/v1、有限正D_filled均值Z、可用/总像素数及组身份；全异常块不进网格，边缘不足3×3保留。q=R_BC*b和角边界启动预计算，四路常驻普通线程避开读取CPU/SMT，同组只构建一次。`pixel`和`region`仍查询全分辨率D_filled，不用块均值代替最小值。v2来源码为camera*blocks_per_camera+block_id，详见[当前块接口/计时](DEPTH_BLOCK3_OPTIMIZATION.md)。

## 第五至第七步当前接口

当前采用用户接受的机体中心虚拟环视：ρ=Z*||b||，规划忽略相机平移/实际近裁面，保留旋转。r=.10m、Lp=3m与.05m档模板膨胀、固定方向/节点来源查表、统一环视和Ω三态在新组接入，不生成重复来源边界，随同一raw/filled保护槽发布。`DepthObservationView::pixel/region`默认filled，可传`DepthImageKind::Raw`使用独立原图摘要。`LocalFieldView::panorama_cell`返回可选ρ、相机/原始完整位与Ω状态；`cell`读取节点状态，`candidate`/兼容`sphere`仅查统一环视，不检查实际相机光心或近裁面。未覆盖/原始不完整保留Unknown，补值不能独自提供自由依据。真实相机位置仍在输入记录，原始`pixel/region`光轴Z查询保留；`block_ray`在虚拟模式返回空。显式configure_real/debug只作历史数值审计。详见[当前模型/API](STEP6_VIRTUAL_CENTER.md)。

`BodyState`新增p_W；`DesiredInput`与`desired_motion`给出d_desired和0～10m/s的v_desired，输入≤1e-9不归一化/速度零。`--desired-source velocity|goal|none`选源，velocity支持`/human_intent`的world/base_link帧；goal接受world帧`/controller/goal` PointStamped，`--goal-speed-mps`默认10。固定2ms与20m/s²参考限幅接口只供后续梯度曲线使用，没有运动输出。详细几何、状态语义、缓存键和验证见[当前块级排除](STEP6_BLOCK_DOMAIN.md)；历史紧凑输入见[0.4.0报告](STEP5_7_COMPACT_INPUTS.md)；原完整来源版见[历史步骤5～7](STEP5_7_LOCAL_FIELD.md)。

## 可复用输入与输出

| 方向 | 话题 | 消息 | 含义 |
|---|---|---|---|
| 输入 | `/human_intent` | geometry_msgs/TwistStamped | 世界系期望运动与偏航意图，来自手柄、键盘或目标协议 |
| 输入 | `/sim/odom` | nav_msgs/Odometry | p_W、R_WB与速度 |
| 输入 | `/controller/goal` | geometry_msgs/PointStamped | world帧目标点，仅desired-source=goal采用 |
| 输入 | `/sim/depth/image_raw`及`camera_info` | sensor_msgs/Image、CameraInfo | 前向深度与相机参数 |
| 输入 | `/sim/depth_left/image_raw`及`camera_info` | 同上 | 左向深度 |
| 输入 | `/sim/depth_back/image_raw`及`camera_info` | 同上 | 后向深度 |
| 输入 | `/sim/depth_right/image_raw`及`camera_info` | 同上 | 右向深度 |
| 输入 | `/clock` | rosgraph_msgs/Clock | 仿真时间，节点使用use_sim_time |
| 输出 | `/position_cmd` | pc_gvf_msgs/PositionCommand | 桥接读取世界系velocity与yaw_dot，时间戳用于新鲜度检查 |
| 桥接输出 | `/sim/cmd_vel` | geometry_msgs/Twist | 现有仿真执行器接收的速度指令 |
| 可选诊断 | `/controller/status`、`/controller/diagnostics` | std_msgs/String | 记录状态与JSON诊断，不提供任何控制决策 |

深度为原Isaac相机的rendered-Z度量，不能直接把所有像素当成欧氏射线距离；应使用各相机实际CameraInfo和坐标系解释。四路水平相机默认320×240，不新增上下相机。具体外参、姿态、话题和帧名以场景与真实运行传感器记录为准。

桥接保留限幅、坐标转换和时间戳/失联检查。无人机在Isaac中的速度响应、加速度和碰撞保护仍存在，这些属于测试平台。新控制器的状态估计使用、感知处理、方向/轨迹生成、反馈、安全检查、制动、恢复、超时与统计逻辑需要重新设计，不能声称旧c36机制已迁入。

## 最小接入方式

新节点必须自行订阅所需输入，发布PositionCommand，不依赖旧`depth_angular_controller`。默认速度上限10m/s、垂向1m/s和a16/tau0.10是继承的测试平台设置，并非新算法已能达到的性能。

1. 在src/new_controller下建立真正的ROS2包，或按需重新组织包名，加入自己的构建定义。该包现已建立，不需再次创建。
2. 先构建新包，确认其可执行文件位于EGO4P0目录内。当前构建脚本已包含`new_controller`，安装的`depth_reader`仅做读取测试；应在后续真正实现运动输出后接入本飞行接口。
3. 用EGO4P0_CONTROLLER_EXECUTABLE指定该可执行文件。可选EGO4P0_CONTROLLER_PARAMS为存在的绝对YAML参数文件。
4. 新launch只添加use_sim_time和你明确指定的参数文件，不注入旧调和场/流盒、缓存、恢复、启动真值白名单和预算参数。
5. 在接入目标/多路线运行前重写相应调度与审计的控制器参数/诊断协议。当前旧调度函数明确拒绝运行，离线审计/统计可继续导入。

没有设置新可执行文件、文件不存在、指向项目外部或旧控制器名称时，入口会在启动Isaac前拒绝。这里没有自动选择旧控制器或直接透传手柄的回退方案。

## 当前可做与尚未做

可继续查看和生成场景、使用消息与桥接代码、复用输入设备和离线统计。当前完成基础设施拆分、第一至第七步输入/预处理/方向与查询、尺寸和局部场输入及期望方向；粗细调和场、梯度曲线/最终包络验收、闭环导航、避障性能、长尾优化、实机验证尚未完成。

历史资料仍保留原路径与版本名。尤其10月22日前20米高Cloud的实验不代表当前40米地图结果，旧0.5–0.8ms观察也不代表新算法计算性能。


2026-11-03历史并行补值版（当前精简版未在线）：生产调用`FrameObservationBuilder::process(frame,rays)`，每路常驻线程执行原始邻域补值→合并块均值/全分辨率区域摘要→方向来源，之后统一归并；`build`保留给已补值图验证。同组不重做补值。`repair_max_ms`为核最大经过时间，包含在`camera_work_ms`中；合并`compression_ns`包含摘要。输入/查询语义和FIFO选择保持。2026-11-03三轮本版在线FIFO已核对：0/15020读取间隔>2.2ms、漏周期0，完整组299/300，在线整组均值1.289/1.736/1.662ms，最长5.032ms；不是严格2ms或相机全部完整保证，见[并行补值实现与验证](DEPTH_PARALLEL_REPAIR.md)。
