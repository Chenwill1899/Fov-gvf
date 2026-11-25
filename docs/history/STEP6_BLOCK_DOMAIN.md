2026-11-03 23:11 +0800 版本说明：本页实际光心0.5.0方案现作为历史审计/冻结基线。用户本轮明确接受[0.6.0机体中心虚拟环视](STEP6_VIRTUAL_CENTER.md)，生产忽略相机平移与真实光心盲区，查询统一ρ环视及Ω；本页数值和原始日志保留，不继承为新模型几何保证。

# 障碍块保守排除：移除新组中的逐近像素几何处理

2026-11-03 22:20 +0800：默认C++实现更新到0.5.0，近障碍改为块级保守排除。读取原有3×3最小深度筛选可能含3m内障碍的块，保留数据并排除其保守方向范围；未排除格默认未知。可用判定留给实际候选位置的原始图完整性与球体投影查询。4包Release构建、CTest8/8、3项ASan/UBSan和12份独立核对通过。密集合成整组均值2.477→1.542ms，但仍有41/300>2ms；本版未运行新Isaac/FIFO，也没有运动输出或完整调和场。

## 默认新组流水线

```text
四路原始邻域补值 → 3×3块/双图区域摘要 → 原有代表观测方向
                                            ↓
                              min_Z ≤ 块内最大Z_limit？
                              ├─ 是：块级保守方向排除
                              └─ 否：保留完整原始远观测依据或未知
                                            ↓
                              四路二维差分 → 前缀恢复 → 相机位合并
                                            ↓
                           2ms读取复用；实际候选按需查询可用性
```

新组构建不再逐近像素执行Z*q+t、角度归格或raw/repaired计数。补值和原图/补图摘要仍必须读取像素，它们不包含逐近像素坐标换算。原本保留的3×3均值代表方向仍生成，所有3m外代表信息不裁掉。同组复用、受保护三缓冲、四路常驻CPU1～4普通线程、CPU5/SMT11避让与预分配保持。

块最小深度来自D_filled，有限正值才参与；为收紧保守范围，在**同一次已有摘要扫描中**增加一个最大Z比较，保存max_Z，不新增图像扫描。原始min_Z/可用数和补图min/max/计数分开保留，原图及补值图不丢弃。块内任一像素若满足0<Z≤自己的3m Z_limit，一定满足min_Z≤block_max_Z_limit；筛选可能多排除，不会因远块均值漏掉细近障碍。

## 保守范围与机体尺寸

相机存在平移，因此不能只把固定相机视线角或块中心格当作整块覆盖。初始化预计算块中心旋转视线q_c、其模长，以及原图块含半像素边缘四角相对q_c的最大偏差Δq。q是像素位置的仿射函数，矩形内任意视线的偏差均被四角最大值覆盖。静态参数随K/T或块配置失效重建。

对当前可能含近障碍的块：

```text
Z_lo = min_Z
Z_hi = min(max_Z, block_max_Z_limit)
Z_mid = (Z_lo + Z_hi)/2
C_B = Z_mid*q_c + t_BC
R = (Z_hi-Z_lo)/2 * ||q_c|| + Z_hi*Δq + 0.10m + 数值余量
```

对块内任何参与3m约束的表面P_B=Z*q+t，由三角不等式有：

```text
||P_B-C_B|| ≤ |Z-Z_mid|*||q_c|| + Z*||q-q_c|| ≤ R-0.10m-余量
```

所以该球覆盖整个可能近表面及机体半径影响范围。不保存三维表面点或点云，只临时计算一个块包围体中心。将球投成保守方位—俯仰区间；若包含机体原点则排除全方向，覆盖极点则覆盖全部方位角；±π接缝拆成两个矩形，粗网格重叠保守合并。角边界增加数值余量后向外取格。

每相机用预分配二维差分数组，每矩形4次写入，完成后恢复布尔覆盖；最后按格合并四相机位。没有按近像素或按每个矩形的全部格逐个累加。无近块时跳过差分与前缀扫描，必要时清除旧工作掩码和旧保护槽内容，防止上一组排除残留。合并仅表示“来自哪些相机的保守排除”，不跨相机取最小光轴Z。

## 不能把剩余区域当自由

每个块的观测依据有三种：

- `ObstacleExclusion`：可能存在3m内表面，整块保守方向范围不可用。数据继续保留供按需回查。
- `CompleteFarObservation`：不含上述近候选，原始像素全部有限正数且raw_min_Z>block_max_Z_limit；仅表示有完整远观测依据，不宣称整个方向格或机体近场可通过。
- `Unknown`：其它情况，包括仍异常、只靠补值、部分原始观测、全异常块。

网格`cell`只返回`Unavailable`或`Unknown`，**不会自动返回Available**。相机位置、视野、近裁面和机体半径使“块完整且无3m内表面”仍不足以证明任意机体系位置自由。实际位置`candidate(center_B)`先查方向排除，再调用已有全分辨率`sphere`投影：完整包络在图内、实际近裁面已知且原始ROI完整/最小Z大于球体远边界，才可给Available；潜在障碍给Unavailable，其余Unknown。补值不自动授予通过依据，缺近裁面或初始盲区证明仍未知。当前位置不重复查询。

相机表面在3m之外也可能影响3m边缘处机体包络，所以CompleteFarObservation不能代替最终尺寸核对；`candidate`仍查询实际包络的完整投影及原始深度。未建立整张最大直行距离图、候选轨迹枚举或导航未知策略。

```cpp
observation_builder.process(frame, pixel_rays); // 新组一次
LocalFieldView view(frame, observation_builder.local_geometry());
auto cell = view.cell(cell_id);                // Unavailable / Unknown + 相机位
auto evidence = view.block_evidence(camera,id);// 块依据，不能当整格自由
const auto state = view.candidate(center_B);   // 实际候选：Available/Unavailable/Unknown
const auto size = view.sphere(center_B);       // 原全分辨率尺寸接口
const auto raw_ray = view.block_ray(camera,id);// 原始条件光轴区间，按需
const auto source = view.pixel_obstacle(camera,u,v); // 精确来源仅按需/诊断
```

Available仅表示该组采集机体系下的观测几何依据，不含数据时效、反馈/响应或最终执行检查，不能直接作为运动下发许可。

这些引用只在保护槽有效期内使用；组序号/时间戳/标定/几何身份和外部坐标/参数/域/边界修订键保持。这里完成候选排除与按需可用性接口，不代表已经构造全部粗细场节点或边界。第八步最终域边界仍应从最终域生成一次；调和求解、梯度参考曲线及第十六步反馈/限幅/响应后的实际预测运动检查尚未实现。

## 相同输入离线对照

| 输入 | 每版次数 | 旧均值 ms | 新均值 ms | 减少 | 新P99 ms | 新最大 ms | 新>2ms |
|---|---:|---:|---:|---:|---:|---:|---:|
| 真实首末静止存档 | 600 | 0.524686 | 0.484856 | 7.59% | 0.781202 | 1.144284 | 0 |
| 密集近障碍（合成） | 300 | 2.477042 | 1.542406 | 37.73% | 2.316386 | 3.482421 | 41 |
| 每块一个细近像素（合成） | 300 | 1.280616 | 1.215333 | 5.10% | 1.873048 | 3.101127 | 2 |

每版三轮交替，100次强制新组构建/输入/轮，预热10次。真实静止存档用已有在线r1首末两组，共600次/版，3m内障碍为0。密集与细近各300次/版为明确数值合成，标定来自该存档：密集Z=1.2m且每17像素原始NaN；细近每块一像素.3m，其余10m。全部编译/CTest/ASan先于计时，普通CPU0协调、CPU1～4工作，没有仿真/GPU渲染负载。

旧版是上次精简版0.4.0冻结ELF，本页同轮旧均值与之前2.394/.512ms等不同，不混成同一轮对照。新版密集仍41/300>2ms、7/300>2.2ms；细近2/300>2ms、1/300>2.2ms。细近P99从1.794升到1.873ms、最大2.480→3.101ms，**长尾未全面改善**。不能用均值证明严格2ms或FIFO读取间隔稳定。

![处理耗时曲线](../../../EGO4P0/docs/performance/depth_block_domain_20261103_220231/block_domain_timings.png)

收益来自工作量与表示方式的变化，输出由精确像素归格统计变为保守块排除，不能说新旧局部格统计完全相同。原始/补值图、代表观测、原始条件射线、精确像素来源与尺寸查询依然可逐字节对照。camera_work_ms包含四路补值/摘要、代表投影、块排除及前缀恢复和等待；shared_projection_local_max_ms与之重叠，不重复相加。local_merge_ms随后合并相机位。

## 保守性的实际代价

真实首组0格被排除；密集合成24240/64800格（37.41%），细近59400/64800格（91.67%）。后者每个块都含近像素，且min/max跨度大，包围球明显扩大方向范围，可能排除实际可行的位置。因此本优化以减少运算换取更保守的候选域，**不能据此声称通行率或导航表现提高**。灰色仍默认未知，不能因为未被排除就通行。

![方向排除掩码](../../../EGO4P0/docs/performance/depth_block_domain_20261103_220231/block_domain_masks.png)

实际首组32682块有完整远观测依据、1558块未知，但旧存档未保存近裁面，实际尺寸查询不能据此给自由。不同相机各自记录排除位，不比较不同光轴的深度选近。

## 验证、诊断和恢复

- 4包Release构建成功；CTest8/8通过；`depth_local_field_check`、`depth_compact_inputs_check`、新增`depth_block_domain_check`三项ASan/UBSan通过，detect_leaks=0，未执行LSan。新增检查180840个随机块内视线/深度及0.10m球面扩张样本，24组随机姿态含极点/接缝/180°粗格，差分掩码与直接矩形并集一致；原312632阈值射线、1200raw ROI、60000球投影与期望/限幅、120组槽复用和零稳态分配检查保留通过。
- 新测试还核对整图异常仍Unknown、原始完整远块、raw孔洞补值后不自由、近细像素触发不可用、实际完整投影才可用、盲区与当前位置未知、旧帧掩码清除和配置失效。
- 最终12份输出独立NumPy核对全部307200像素阈值/q、每块min/max/原始完整性及三态依据、独立球体角区间/差分并集；每个真实近像素及六个0.10m轴向偏移均被排除。原raw/filled/b/d、代表方向、块CSV、摘要/查询、停止后诊断精确像素CSR、原始射线区间与尺寸查询共18份payload逐字节等于同轮0.4.0；只改变新候选排除掩码。
- 局部导出版本`ego4p0_local_field_v3_block_exclusion`，新增`*_block_domain.csv`和`*_local_excluded_camera_mask.bin`。精确像素CSR仍只在结束后诊断重建，新组热路径计数`hot_near_pixel_transforms=0`。日志明确改为local_excluded_blocks/local_excluded_cells，不冒充精确障碍像素数。完整原图/补图和全分辨率按需查询保留。
- 33项源SHA、16项构建身份、reader/benchmark/check ELF冻结；[运行记录](../../../EGO4P0/docs/performance/depth_block_domain_20261103_220231/execution.json)、[独立复核](../../../EGO4P0/docs/performance/depth_block_domain_20261103_220231/independent_comparison.json)、[汇总](../../../EGO4P0/docs/performance/depth_block_domain_20261103_220231/summary.json)和全部构建/测试日志保留。两张图已目视核对；7项Python AST、19个当前文档本地链接、launcher --help及一份本轮0.4.0输出的新版验证器兼容检查通过。
- 开发初次独立新测试有一处误导缩进警告，已修正；已记录并补上“全异常块计入未知”边界与无近块时的惰性清理，最终重新构建和验证。初始/最终日志全部保留，无遗漏的失败测试。构建中既有Python禁止byte-compile提示仍保留。
- 本机RTX5070 GPU0/cuda:0、相机500仿真Hz、.02倍/.2s、2ms现实读取与FIFO10＋200μs选择均保持；本轮未改系统权限、场景或源EGO2P6，也未启动新Isaac/FIFO。此前在线结果属于旧冻结ELF，不能继承为本版成绩。

源码：[块保守球/角区间](../../src/new_controller/include/new_controller/depth_block_domain.hpp)、[四路差分与合并](../../src/new_controller/include/new_controller/depth_local_field.hpp)、[候选/尺寸查询](../../src/new_controller/include/new_controller/depth_size_query.hpp)、[随机几何测试](../../src/new_controller/test/depth_block_domain_check.cpp)。

修改前保存点：`/home/starry/isaac-data/HISTORY/snapshots/EGO4P0_block_domain_20261103_220231/before.tar.gz`及before_sha256.json，HISTORY环境版本记录另存additional_before.tar.gz/sha。恢复需原子替换项目文件、移除本轮新增实现/测试/分析文件并重建，HISTORY和项目工作日志只追加恢复记录，不覆盖旧记录。结果目录保留旧/新ELF、源码、身份、全部对照输出和编排/分析脚本。

```bash
cd /home/starry/isaac-data/EGO4P0
EGO4P0_BUILD_TESTING=ON bash scripts/build_isaac_ros_workspace.sh
ctest --test-dir docs/performance/runtime/build/new_controller --output-on-failure
# 仅离线，NEW_OUTPUT必须不存在：
# docs/performance/runtime/build/new_controller/depth_local_field_benchmark ARCHIVE first NEW_OUTPUT on
PYTHONNOUSERSITE=1 /usr/bin/python3 -B scripts/verify_local_field.py OUTPUT
MPLCONFIGDIR=/tmp/ego4p0-mpl-profile PYTHONNOUSERSITE=1 /usr/bin/python3 -B scripts/analyze_block_domain.py /home/starry/isaac-data/EGO4P0/docs/performance/depth_block_domain_20261103_220231
```
