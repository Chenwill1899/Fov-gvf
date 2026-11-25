2026-11-03 22:20 +0800 版本更新：本页0.4.0逐近像素统计已由[0.5.0块级候选排除](STEP6_BLOCK_DOMAIN.md)替换；本页计时与API统计含义作为冻结基线保留。当前cell是保守不可用/默认未知，实际candidate查询才能给可用，不再构建精确逐近像素统计。

# 局部场输入精简：取消自由端点索引与重复来源边界

2026-11-03 19:07 +0800：已将用户要求接入默认C++热路径，包版本0.4.0。4包Release构建、CTest7/7、两项ASan/UBSan、12份输出独立数值核对及冻结旧/新版本对照完成。真实静止存档整组均值1.196→0.512ms；密集近障碍合成输入6.997→2.394ms，仍超过2ms。**本版未运行新Isaac/FIFO仿真或运动输出**；此前[0/15020读取间隔>2.2ms](DEPTH_PARALLEL_REPAIR.md)属于旧版冻结ELF。

## 默认数据与处理位置

- 删除自由射线端点P_B换算、方向归格、计数/前缀和/来源散写，以及逐组free_rays数组。原图块范围、原始最小Z/可用数、近裁面与3m静态阈值保留。`block_ray(camera,block_id)`只在消费时返回光轴区间，不再给区间端点分配角格。
- 第三步保留所有距离的3×3代表观测及相机信息，默认只生成每格代表数量/相机位；缓存每块的方向格与局部范围标记。默认无完整代表来源CSR，查询块按相机/id进行；`configure_debug`可显式开启代表CSR，普通生产入口不启用。仅用于CSR写出的临时块id列表默认也不分配、不写入。
- 局部输入改为每方向格的原始/补值约束像素数量与相机位，不保留整组局部像素来源CSR。每相机预分配私有计数/已触及格表，归并仅访问触及格；旧槽的约束只清理其上次触及的格，避免遗留。它们是观测统计，**不是可用／障碍／未知求解节点标签**。
- 删除基于来源分布的无证据/不完整/观测边界扫描与缓存。第八步须根据真实投影几何和最终求解域生成一次边界；本次没有虚构尚未定义的节点状态、边界类型或系数。
- 同一块的P_B、格号、局部范围标记在原代表循环内一次生成；局部近像素查格以已有块均值格号为搜索起点，仍通过预计算角边界精确判定并在必要时回退。中心像素和均值相同时直接复用格号。近障碍仍逐原像素核对自己的Z_limit；远块均值中的细近像素由全分辨率min_Z兜底，不被裁掉。
- 四路热计数放进各自`alignas(64)`工作结构，结束后各提交一次；不再逐像素更新相邻的共享统计字段。它减少潜在缓存行争用，未用硬件计数器单独证明各项收益。

保持四路常驻CPU1～4普通工作线程、受保护三缓冲和重复组复用。新组执行“补值→双图块/区域摘要→共享代表/局部核对→紧凑计数归并”；2ms读取只消费最新完成组。CPU5及其SMT11不供工作线程使用。固定r=.10m、Lp=3m、2ms、期望速度0～10m/s及20m/s²参考限幅未改变；不改本机RTX5070 GPU0/cuda:0、500仿真Hz、.02倍/.2s设置。

## 几何和证据语义

`block_ray`返回原图半开块范围和`[near_clip_Z,min(raw_min_Z,block_min_Z_limit))`的条件光轴区间。无原始有效像素时没有raw_min_Z；缺近裁面时begin_known=false，不能把零起点当已观测自由。原始不完整块必须按需用全分辨率查询核对，射线来源数量或端点位置都不能证明方向格自由。

`pixel_obstacle(camera,u,v)`按需返回3m内精确局部像素来源；`sphere(center_B)`仍按真实相机投影和全分辨率raw/filled区域查询核对球体。原图和补值全图、各自摘要、相机标定和b/d表保留：补值可增加保守障碍约束，不能自动作为自由证据；不跨相机比较光轴Z取最小值。近场无显式初始依据时保持未知，原点不重复检查。

完整来源诊断在停止计时/停止读取后，由已缓存块格号和图像重建导出；默认处理期间没有这些CSR前缀和或散写。原有方向诊断格式`ego4p0_direction_queries_v2_blocks`继续兼容，标记hot_full_provenance=false；局部导出升级为`ego4p0_local_field_v2_compact`，无local_free_offsets/sources、无旧边界文件。导出的完整障碍CSR仅用于核对，不回到热路径。

## C++消费接口和缓存失效

```cpp
observation_builder.process(frame, pixel_rays);  // 新组一次；同组复用
DepthObservationView depths(frame);
LocalFieldView inputs(frame, observation_builder.local_geometry());
auto compact = inputs.cell(cell_id);             // 数量/相机位，不是节点自由标签
auto block = depths.block(camera, block_id);     // 代表观测、范围及组身份
auto ray = inputs.block_ray(camera, block_id);   // 按需读取条件光轴区间
auto close = inputs.pixel_obstacle(camera,u,v);  // 按需精确近像素来源
auto envelope = inputs.sphere(center_B);         // 全分辨率尺寸依据
const auto key = inputs.input_key(coordinate_revision, parameter_revision,
                                  domain_revision, boundary_revision);
```

`DepthObservationView::observation(cell,k)`在默认紧凑模式返回空，不隐藏重建或扫描；需完整调试来源时初始化使用`configure_debug`。每块仍有O(1)回查入口。上述引用只能在同一保护槽有效期间使用，不能跨槽持有。

新`FieldInputKey`包含同步序号/时间戳、观测与几何配置身份，及消费方显式传入的坐标/参数/求解域/边界修订号。新组或相机K/T/近裁面变化会改变身份或使旧视图失效；未来使用方必须在坐标关系或参数变化时更新修订号。当前没有求解器，不假装已经实现求场缓存。后续按消费时机落实：域或边界变化才更新场输入并求场，选中参考曲线才积分梯度，每2ms推进参考/反馈/必要执行检查。

本次没有加入假想轨迹、候选枚举、整张逐方向最大距离图或重复停车计算。第十六步仍需实现加入反馈、限幅和响应后的**实际预测运动**包络检查；当前仓库尚无该完整执行检查器，也未实现粗细调和场或梯度参考曲线，不能用本步观测统计代替它们。

## 相同输入冻结对照

| 输入 | 次数/版 | 旧均值 ms | 新均值 ms | 减少 | 新P99 ms | 新最大 ms | 新>2ms |
|---|---:|---:|---:|---:|---:|---:|---:|
| 真实首末存档：局部障碍0 | 600 | 1.196324 | 0.512393 | 57.17% | 0.818728 | 1.646469 | 0 |
| 合成密集近障碍：307200像素 | 300 | 6.996819 | 2.394165 | 65.78% | 3.115852 | 3.959815 | 300 |
| 合成细近障碍：34240像素 | 300 | 2.201543 | 1.221637 | 44.51% | 1.700620 | 2.122444 | 1 |

每种版本三轮交替执行，100次强制新组构建/输入/轮，预热10次。真实输入使用既有在线r1首末两组（共600次/版）；两种合成深度使用同一真实标定（各300次/版），明确没有启动仿真。普通调度CPU0协调、CPU1～4工作；所有编译/CTest/ASan在计时前完成。旧版使用前次步骤5～7的冻结ELF，新版为本任务最终冻结ELF。不同任务以前测到的1.173ms和本次1.196ms不混作同轮基线。

真实初始存档3m内局部障碍为0，33302条原始射线区间可供按需读取，因此不能把0.512ms推广到所有环境。密集合成：Z=1.2m且每17像素一个原始NaN，补值后307200近像素；细近合成：每3×3块一个.3m原像素，其余10m，34240近像素且块均值多在远处。两种输入同时覆盖原始/补值来源区别和均值漏检风险。

密集测试新版300/300>2ms；细近测试新版1/300>2ms、0/300>2.2ms。真实600次本轮没有>2ms，但中间版本同类测试出现3.490ms长尾，短离线通过不构成长期实时保证。这是**新组准备成本**，不等于独立FIFO读取间隔、GPU渲染、ROS接收延迟或未来完整控制耗时。

![整组处理耗时曲线](../../../EGO4P0/docs/performance/depth_compact_inputs_20261103_183037/compact_input_curves.png)

![分项均值](../../../EGO4P0/docs/performance/depth_compact_inputs_20261103_183037/compact_stage_means.png)

camera_work_ms包含四路补值、压缩/摘要、代表投影/局部核对和协调等待；index_merge_ms与local_merge_ms随后顺序执行。新版shared_projection_local_max_ms与camera_work重叠，且包含原代表投影，不可与旧local_camera_max_ms当成同一核比较或重复加总。详见[机器汇总](../../../EGO4P0/docs/performance/depth_compact_inputs_20261103_183037/summary.json)与[24次最终运行记录](../../../EGO4P0/docs/performance/depth_compact_inputs_20261103_183037/verified_execution.json)。

局部动态数组每保护槽元素容量从3,109,448降至1,036,800字节，减少66.66%；计算含原局部CSR/free_rays/边界数组与新Cell/touched数组。本机sizeof(FreeRayEvidence)=32、sizeof(Cell)=12。此值**不包括**图像、静态表、工作线程私有摘要、容器本身或进程RSS，不能称总内存减少66.66%。

## 验证与保留的失败

- `EGO4P0_BUILD_TESTING=ON bash scripts/build_isaac_ros_workspace.sh`最终4包成功，既有Python禁止byte-compile提示保留；`ctest --test-dir docs/performance/runtime/build/new_controller --output-on-failure`7/7通过。两项改动测试ASan/UBSan通过，detect_leaks=0，未执行LSan。
- 原312632阈值射线、1200raw ROI、60000球面投影、细近像素/原始孔洞/盲区/初始证明/期望方向与1000参考限幅、零稳态分配检查保留并通过。新增120组经受保护槽轮换，核对7,776,000格清除、同组复用、保留远粗观测、四类外部修订及K/T配置失效、调试/紧凑模式一致与零稳态分配。
- 最终12份新版输出经独立NumPy核对全部307200像素阈值/q、每格raw/repaired计数/相机位、真实像素归格、按需射线区间、1200raw摘要、8球查询和诊断CSR；18份同名图像/表/来源/查询文件与各自旧版逐字节一致，真实组的原步骤1～4数据也与最初在线存档逐字节一致。见[独立复核记录](../../../EGO4P0/docs/performance/depth_compact_inputs_20261103_183037/independent_comparison.json)。
- 最终31项源码SHA、15项运行构建身份和reader/benchmark/check ELF冻结；最终运行记录确认源未变化。Python AST与launcher --help另记在validation.json。`depth_reader --help`本身不支持无值选项，探测退出3、报missing argument value，未把它写成启动或在线通过。
- 开发中两条ASan编译命令误写test为tests而失败，日志保留，路径修正后两项最终编译/运行均通过。中间私有计数版12份输出数值与旧版一致，但其校验末尾源码SHA检查时恰逢继续删除临时来源id列表，整体返回失败；保留原记录，不算最终通过。停止源码修改后重新冻结、24次计时及12份独立校验全部完成并通过。
- 探索pilot、mean_hint中间版本、private_counters中间版本的源码/ELF/原始输出及长尾均保留。中间密集均值5.115→2.365ms提示私有计数有收益，但轮次/系统状态变化，不能把差值当硬件级因果剖析。最终以本页同轮冻结旧/新对照为准。

## 复现和恢复

```bash
cd /home/starry/isaac-data/EGO4P0
EGO4P0_BUILD_TESTING=ON bash scripts/build_isaac_ros_workspace.sh
ctest --test-dir docs/performance/runtime/build/new_controller --output-on-failure
# 离线单例；输出目录必须尚不存在，不启动Isaac：
# docs/performance/runtime/build/new_controller/depth_local_field_benchmark ARCHIVE first NEW_OUTPUT on
PYTHONNOUSERSITE=1 /usr/bin/python3 -B scripts/verify_local_field.py OUTPUT
MPLCONFIGDIR=/tmp/ego4p0-mpl-profile PYTHONNOUSERSITE=1 /usr/bin/python3 -B scripts/analyze_compact_inputs.py /home/starry/isaac-data/EGO4P0/docs/performance/depth_compact_inputs_20261103_183037
```

修改前保存点：`/home/starry/isaac-data/HISTORY/snapshots/EGO4P0_compact_inputs_20261103_183037/before.tar.gz`及before_sha256.json。恢复按需原子替换项目源码/配置并重新构建；HISTORY和项目日志只追加恢复记录，不回滚覆盖已有历史。[结果目录](../../../EGO4P0/docs/performance/depth_compact_inputs_20261103_183037)保存旧/新ELF、构建日志、源码身份、冻结源、所有正式/探索/失败输出、图表及编排脚本，不依赖/tmp恢复。未修改HISTORY规则、源EGO2P6、场景、系统调度权限或GPU配置。
