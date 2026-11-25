> 2026-11-04 17:30 +0800：**历史实现，已按用户要求撤回，不属于当前活动版本。** 本报告和全部实验数据保留供追溯；源码已移到/home/starry/isaac-data/HISTORY/snapshots/EGO4P0_rollback_steps89_20261104_172926。当前使用[虚拟中心环视0.6.0](STEP6_VIRTUAL_CENTER.md)，回退验证见[记录](../../../EGO4P3/docs/ROLLBACK_STEPS8_9.md)。

# 第八、九步：两级球面调和场与单条角域参考曲线

2026-11-04 13:40 +0800，C++17 / new_controller 0.7.0。已实现并通过离线验证；没有启动新版Isaac、FIFO在线测试或运动执行。保持虚拟中心模型、r=0.10m、Lp=3m，以及5070/500仿真Hz/0.02倍/0.2s/现实2ms/FIFO10＋200μs既有选择。

## 实现与输入

生产路径：新完整同步组完成既有处理后，接收线程从第一层摘要汇总第二层，复制紧凑方向状态到独立SPSC三缓冲；场线程取最新快照和期望方向/q0；2ms线程只提交标量并取得完整发布版本。没有借用已释放的图像槽，不在读取线程求场、复制势数组或写日志。场线程显式SCHED_OTHER、优先级0，CPU允许集合剔除读取核及SMT同胞（本机5/11）。四路原有工作线程和16份既有输入/深度头文件字节保持。

| 图像层 | 每相机尺寸 | 数据 |
| --- | --- | --- |
| 第一层 | 107×80 | 既有3×3块、有效均值Z、最小ρ、原始有效数 |
| 第二层 | 36×27 | 第一层再3×3汇总，共3888块/四路，边缘保留 |

第二层mean_z按第一层valid_pixels加权；第一层mean_z保持已有float32语义，第二层使用double累积。保留min_rho、raw_min_rho、补值后有效数、原始有效数和真实总像素数；空均值/最小值为NaN。只读块摘要，不再扫描原图。均值不授予通行，原始缺失与无覆盖仍为未知。

q0输入优先级为`/controller/reference_tangent`（Vector3Stamped，base_link，有限非零），否则当前v_B；静止时采用d_desired作为初始锚向。期望输入仍由第七步产生。近零期望返回Idle/no_direction，不归一化；q0已在所选出口帽内返回Idle/exit，仅有一个锚点，valid=false表示未求梯度，不能当成已求出的场。q0未知或受障碍影响则返回NeedsRefinement，不自动挪动生产起点。

## 粗场与局部细场

粗网格120×60（3°），细网格360×180（1°）。粗节点必须对应9个可用细角单元，并有原始完整远观测依据；第二级近障碍也会阻断。粗层混合单元不直接认定无路。保留Unknown/Unavailable/Available三态，求解域仅取Available。

粗场只选与q0相关的可用连通分支。出口选分支内最接近期望方向的可用节点；期望方向未落在此分支时，exit_clipped=true，出口只是已观测分支中的局部出口，不代表到达世界目标或许可进入未知。低势帽选此分支中远离出口的一侧。默认两帽半径3°，小分支自动缩至分离角的0.24倍；先判出口再判低势，0/1绝不重叠。粗层边界帽按节点中心标记，继承到细格后包含粗格量化误差，不能把出口终止解释成精确指向目标。

两层均离散用户给定的球面方程。有限体积面系数为：

- 经度边：Δβ/(cosβ·Δα)，经度周期连接；
- 纬度边：cosβ_face·Δα/Δβ，极面面积为零；
- 障碍/未知邻接不加入通量，得到零法向通量；0/1帽为Dirichlet。

使用对角预条件共轭梯度，固定邻接、方向、度量与系数预计算。停止残差是`max |Aφ-b|/diag(A)`，阈值1e-7，并重新计算真实残差；它是离散方程的归一化节点残差，不是连续PDE误差上界。每次最多1600次迭代；无Dirichlet锚、迭代不收敛、非有限数或明显越过[0,1]都不标记成功。

初始细域覆盖细层可达路径、粗场走廊和默认12°扩展区域。人工细域边界仅在四个插值来源均属于有效粗场时使用双线性I(φc)；接触粗层0/1条件时保留0/1。真实障碍/未知边界仍零通量。粗层不连通但细层存在通道、q0粗格混合、耦合无效或积分触及人工边界时扩大到已知的相关细连通分支。绝不把粗层障碍前方当成提前出口，也不切换到粗梯度拼接。最坏会扩大到整个相关分支，因此本版并非所有场景都能保持很小的细域。

相同组和意图不重复提交任务；新组仍重建域/边界，确认完全相同后才复用势值。域/边界改变用旧势热启动；标定/几何身份、坐标修订、R_WB或p_W变化使缓存失效。配置在configure时固定，重配重新分配并重建。缓存比较实际域标记与Dirichlet值，不仅依赖哈希。热启动在本次大分支中收益有限，不能据此声称明显加速。

## 一条曲线与查询

只在经过的位置求F=G^-1∇φ，不生成全图梯度或归一化场。普通区域用与有限体积一致的面通量重建，法向分量在未知/障碍面处趋零；极区用三维切平面局部导数和球面指数映射，local_chart=true时F_alpha/F_beta为NaN，读取gradient_B和norm2。

按正梯度做受控步长中点积分，最大0.4°、最小0.002°、最多2048步，步长对应可变Δτ，不把控制速度当积分参数。每段递归检查大圆中间位置和格角的保守覆盖；经度接缝周期处理，穿越极点须有完整相关极行。不会跳过未知/障碍。先判断Exit，再用norm2≤ε_F²（ε_F=1e-7）判断停滞。耦合边界返回NeedsExpansion并重新求细场，最多一次整分支扩展；之后仍不能推进则如实输出。

状态分开读取：FieldStatus=NoInput/Valid/NeedsRefinement/SolveFailed/Idle；CurveEnd=NoField/Exit/Stagnation/NeedsExpansion/Blocked/StepLimit/NoDirection。Valid只说明场求解有效，不能单独当作参考曲线完成或执行许可。复用时本次积分步数和梯度查询为0，但curve_size和整条缓存曲线保留。

## 离线验证与耗时

4包Release构建、CTest11/11通过；新两项检查分别通过ASan＋UBSan（detect_leaks=0，未做泄漏检测）。数值检查覆盖6000个解析经度解节点、非恒定纬度解（含-tanβ项）、残差/迭代拒绝、纯Neumann拒绝、固定域零分配、边界互斥、局部耦合、粗层遮挡窄通道、粗分支断开、零场、极区积分、接缝与未知中间位置。异步检查覆盖持有槽不可覆盖、最新快照、仅意图变化触发、正常调度及退出。

独立NumPy核对7份导出：二级加权均值/双最小ρ/原始计数与边缘总数、球面离散残差、有效插值来源与值、继承0/1、曲线单位长度/切向梯度/度量范数及每段101个中间位置。原始/补值图用于核对，合成angular_*只代表给定角域数值夹具，不是新仿真或相机几何覆盖成绩。

真实数据取历史FIFO r1首/末帧。预设q0=(-20°,0°)均为Unknown，real_first/last共600个计时样本均返回NeedsRefinement/no_field，未强行补成自由。另将离线锚点选为已有Available的(-20.5°,1.5°)，supported_first/last均在16312节点细分支收敛（789次，残差约9.6153e-8），158步到出口；这是离线输入选择，生产代码没有此自动移动起点操作。真实期望方向(20°,0°)也需裁到可观测出口，exit_clipped=true。

每个case×mode有100次计时，另3次预热，共2100个正式样本。cold每次清缓存；boundary_change交替16°/20°；new_group_reuse仍更新摘要/域，确认同域后复用。序号和时间戳为合成回放身份，不是新采集节拍。普通调度，计时期间无构建/测试；不包含相机渲染、ROS传输或2ms读取唤醒。接收端capture在求场总耗时之外单列，真实约0.09～0.12ms/次。

| 有效起点/场景 | 冷算均值ms | P99 ms | 最大ms | 边界变化均值ms | 新组同域复用均值ms | 复用最大ms |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| supported_first | 71.1873 | 71.6082 | 71.9544 | 71.1197 | 0.2961 | 0.3209 |
| supported_last | 71.3574 | 71.4815 | 71.5572 | 70.9490 | 0.2963 | 0.3119 |
| angular_open | 1.2290 | 1.2403 | 1.2416 | 1.1247 | 0.2604 | 0.2761 |
| angular_gap | 5.7325 | 5.7548 | 5.7760 | 4.8397 | 0.3607 | 0.3813 |
| angular_narrow | 0.2834 | 0.2880 | 0.2978 | 0.2837 | 0.1631 | 0.1853 |

**真实大分支冷算仍约71ms，显著超过2ms；主要耗在细场CG。** 同域复用约0.30ms也不是读取循环的耗时。2ms线程与求场分离，完成前可读上一完整版本及其组/年龄；本版尚无在线CPU竞争、数据年龄或FIFO间隔成绩，不保证每2ms都有新场或严格实时。当前0.02倍下新图理论现实间隔约100ms，不可外推到现实500Hz新图。后续可针对混合粗格导致的整分支细化继续优化。

![细场与曲线](../../../EGO4P0/docs/performance/harmonic_20261104_130556/release/benchmark/fields_and_curves.png)

![求场耗时](../../../EGO4P0/docs/performance/harmonic_20261104_130556/release/benchmark/field_timings.png)

## 文件、版本与复现

新增头文件harmonic_types/solver/curve/planner/worker/io.hpp。生产在depth_reader.cpp接入；输出coarse_image.csv、coarse_field.csv、fine_field.csv、reference_curve.csv、harmonic_meta.json、harmonic_jobs.csv。boundary取值0=域外、1=内部、2=低势、3=出口、4=耦合；state沿用0=Unknown、1=Unavailable、2=Available，域外不等于未知。势与曲线附输入组、标定/几何/参数身份，reads.csv另有field_version、field_sequence、field_age_ms、field_status、curve_termination、field_compatible。field_compatible仅验证版本/输入相符，不是执行许可。

HarmonicConfig提供容差、迭代上限、帽/走廊、步长等C++配置；reader可用--field-tolerance、--field-max-iterations、--field-corridor-deg、--field-cap-deg。其余输入与管理员脚本不变。第八步要求当前1°输入，其他输入角分辨率明确拒绝，避免静默错误映射。保存实际启动参数以便复现；本轮基准全部使用默认配置。

```bash
cd /home/starry/isaac-data
EGO4P0_BUILD_TESTING=ON bash EGO4P0/scripts/build_isaac_ros_workspace.sh
ctest --test-dir EGO4P0/docs/performance/runtime/build/new_controller --output-on-failure
PYTHONNOUSERSITE=1 MPLCONFIGDIR=/tmp/ego4p0-harmonic-mpl /usr/bin/python3 -B EGO4P0/scripts/verify_harmonic_fields.py /home/starry/isaac-data/EGO4P0/docs/performance/harmonic_20261104_130556/release/benchmark --plot
```

最终冻结：/home/starry/isaac-data/EGO4P0/docs/performance/harmonic_20261104_130556/release。42份源码/配置/校验脚本SHA256、24项运行身份、4个执行ELF和压缩源码均保留；实际命令在benchmark_command.json。恢复点：/home/starry/isaac-data/HISTORY/snapshots/EGO4P0_harmonic_20261104_130556/before.tar.gz；恢复前检查当前改动，新加harmonic_*文件需单独处理。

历史尝试也保留：初次梯度核对失败后修正查询位置换算；两次构建分别因测试auto多变量类型不一致、基准输出流括号错误失败，之后已通过。final/目录的旧梯度对两个可用真实起点只积分14步后Blocked；final_flux/及release/改为零通量面重建后均到出口。final不是当前最终版，**以release/为本次最终证据**。新增已对齐锚点的坐标/梯度有效性检查，Idle不伪造已求梯度。没有删除失败或用旧在线成绩认证新版本。

本版仅到角域参考；空间参考曲线、速度推进、反馈/限幅/响应后的实际预测包络检查和运动输出仍未实现。既有10m/s、20m/s²限制接口保留，但不是闭环控制验收。没有点云、环境地图、候选轨迹枚举或整张最大直行距离图。
