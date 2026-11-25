2026-11-03 13:53 +0800 当前实现已按用户要求升级为[3×3块代表/旋转视线/四线程/角边界v2](DEPTH_BLOCK3_OPTIMIZATION.md)。原图与精确区域查询保留，方向来源改为相机+块范围/均值；本文下面v1逐像素格式及9ms结果作为历史保留，不能用v1来源码解码v2。最新离线0.646ms，新在线FIFO未运行。

2026-11-03 13:19 +0800 新增[约9ms耗时位置诊断](DEPTH_INDEX_PROFILE.md)：原实现本轮均值9.235ms，融合分段9.328ms；逐像素变换/角度/归格占91.07%，分块两次atan2合计4.732ms。不同循环计时不可直接相加；此处旧实测保留，生产实现与运行参数保持，新在线未运行。

# 第三、第四步：机体系二维方向网格与深度查询

2026-11-02 23:47 +0800。用户确认初始角分辨率为**方位角1°×俯仰角1°**。C++模块已接入每组同步深度的接收端处理，四包Release构建与CTest3/3通过；真实Isaac存档首末图的全来源、方向、覆盖、区域摘要及查询返回均经独立离线核对。新版本尚未运行Isaac/FIFO在线仿真，当前性能只描述离线缓存数据。

## 第三步：方向对应与保留来源

对补值后仍有限且>0的像素，严格使用未归一化b：

```text
P_C = Z * b(u,v)
P_B = R_BC * P_C + t_BC
alpha = atan2(P_B.y, P_B.x)
beta  = atan2(P_B.z, hypot(P_B.x, P_B.y))
```

P_C和P_B只在当前像素计算时临时存在，不保存空间点或点云。保留平移t_BC，不能只旋转单位视线d：同一像素在不同深度时相对机体原点的方向可能不同。

角域为`alpha∈[-180°,180°)`、`beta∈[-90°,90°]`，1°网格为360列×180行，共64800格。+180°与−180°合并在接缝首列；上极点归入最后一行；恰在俯仰极点的方位角取0。除顶部β外，单元采用左闭右开边界。配置非整除角分辨率时，末单元截在域边界，不暗改步长。最多1048576格，防止不合理配置触发无界内存申请。

每个单元用连续来源列表关联**全部**像素，不比较不同相机的光轴Z，不选一个“全局最近深度”。来源码`camera_index*(width*height)+v*width+u`可从同一保护帧还原相机、u/v、Z、同步组时间戳和序号；Z仍在该帧D_filled中，组信息共享存储，查询时一并返回。内部相机编号0～3分别为Front/Left/Back/Right，对应公式中的第1～4路。

原始D、补值D_filled、K/T_BC与b/d继续保留。残余NaN/±Inf/零/负值不进入方向表，原图数值不改。若变换结果非有限或恰在机体原点、方向未定义，该来源只从方向表排除并计数，数字深度查询仍按原值判定；本轮真实数据的几何排除数均为0。

### 无观测与相机覆盖

`offsets[cell+1]-offsets[cell]`为该格来源数，0表示无观测，不填成已有观测，不跨盲缝插值。单元的相机来源位为Front=1、Left=2、Back=4、Right=8，可同时出现多位；从中查询每路本组占据哪些角单元和占据格数。

这里的覆盖是**当前补值图中可用数值对应的表面方向覆盖**，包括按第二步规则补值成功的像素。它不是相机标定FOV的几何保证，也不把FOV内没有有限深度的方向标成已观测；几何标定视野、内参和外参仍在`simulation.sensors.json`中。空格和有格都不直接标记为调和自由域或导航许可。

真实存档首末样本各保留298795个来源，22966格有观测、41834格无观测；没有重复/丢失/虚构来源。每像素只进入其自身方向单元，单元内可有多个像素及多台相机：

| 相机（内部编号） | 来源像素数 | 有观测的角单元数 | 无定义几何排除 |
|---|---:|---:|---:|
| Front (0) | 76396 | 5784 | 0 |
| Left (1) | 74426 | 5846 | 0 |
| Back (2) | 71599 | 5572 | 0 |
| Right (3) | 76374 | 5764 | 0 |

本轮真实场景的单元恰未出现多相机重叠，重叠保留行为由四相机同格、不同Z的C++检查验证；没有通过丢弃较大的Z制造单来源。

![二维方向与相机观测覆盖](../../../EGO4P0/docs/performance/depth_steps34_20261102_232509/direction_coverage.png)

[方向覆盖PDF](../../../EGO4P0/docs/performance/depth_steps34_20261102_232509/direction_coverage.pdf) · [完整格偏移](../../../EGO4P0/docs/performance/depth_steps34_20261102_232509/final_last/sample_direction_offsets.bin) · [全部来源码](../../../EGO4P0/docs/performance/depth_steps34_20261102_232509/final_last/sample_direction_sources.bin) · [相机覆盖位](../../../EGO4P0/docs/performance/depth_steps34_20261102_232509/final_last/sample_direction_coverage.bin)

## 第四步：单像素与区域查询

实现位于[depth_observations.hpp](../../src/new_controller/include/new_controller/depth_observations.hpp)。返回`DepthQueryResult`，包括状态、可选Z值、相机编号、区域、可用/总像素数、组时间戳和序号。

- `pixel(camera,u,v)`：返回D_filled对应像素的光轴Z。图外、非法相机或仍异常时，Z为`std::nullopt`；没有虚构有限深度。
- `region(camera,{u0,v0,u1,v1})`：区域为整像素半开矩形`[u0,u1)×[v0,v1)`。只取该相机已有有限正Z的最小值，返回参与像素数和区域总像素数。图外不隐式裁剪，反向/空矩形分别报告错误/空区域。
- 没有可用数值时返回`no_usable_depth`且Z为空，不返回最大量程、无穷大或替代有限值。
- 部分像素可用时，Z仅描述这些可用值；覆盖计数不证明整个区域自由，也不是轨迹碰撞判定。
- **不构建逐像素V_i掩码，不计算U_i(Q)**，不规定未知区域导航策略，不生成候选轨迹或调和场自由域。

区域加速索引按每相机16×16像素分块，预计算最小有限正Z与可用数值计数。完全包含的块直接用摘要；只扫描与查询边界相交的部分块。320×240完整图查询读取300个摘要、0个原像素，结果与直接扫描76800像素相同。没有用分块最小值代替部分块的精确最小值。

```cpp
#include "new_controller/depth_observations.hpp"
const ego4p0::DepthObservationView query(*input.depth);
if (query.ready()) {
  auto pixel = query.pixel(0, u, v);                    // Front
  auto roi = query.region(0, {u0, v0, u1, v1});
  if (roi.z) {
    float optical_z_min = *roi.z;
    // roi.valid_pixels / roi.total_pixels are numeric coverage only.
    // roi.stamp_ns and roi.sequence identify the same protected group.
  }
  auto cell = query.index().layout.cell(alpha_rad, beta_rad);
  if (cell) {
    auto n = query.index().offsets[*cell+1] - query.index().offsets[*cell];
    for (uint32_t k=0; k<n; ++k) {
      auto source = query.observation(*cell, k);
      // source: camera, u, v, optical_z, stamp_ns, sequence
    }
  }
}
```

来源列表内部是预分配容量，只有`[0,source_count)`和各单元offset范围有效；使用上面的查询接口不会读到未使用的尾部。所有视图随当前帧所有权有效，不跨下一次`latest()`保留。组序号或时间戳不匹配时报告`index_not_ready`，不将旧索引用于新图。

## 接入、时间与内存

C++普通接收线程依次完成：严格四路同步→原始图复制→第二步补值→第三步方向来源索引→第四步区域摘要→发布同一个三缓冲槽。原始图、补值图、方向来源、区域摘要共享同一组身份和所有权，不存在先发布图像再让控制线程等待索引的中间状态。

2ms线程按已选FIFO10＋200μs方案读取最新完整组，查询缓存，不重建索引。新帧没有完成时继续使用最新完整组，数据年龄仍需检查。新索引平均约9ms/组，**不能把整组建索引称作2ms完成**；当前0.02倍速下相机组现实目标约100ms一组，构建与2ms读取分开。未来提高现实新帧率时需重新测算接收处理预算。

所有来源、格偏移、摘要和构建暂存空间在启动时分配。每个槽只存有界索引和当前图像，控制线程查询及接收端稳态构建均无动态分配；单独保留首组验证快照，不保存全部历史图像。没有保留P_C/P_B三维点集。配置的角分辨率影响内存和构建成本，默认1°已明确记录为用户选择。

两种离线首末样本（原真实Isaac会话181620）分别10次预热后100次缓存建索引、2000次单像素和区域查询；CPU5、普通调度，未隔离其它桌面任务，部分测量与构建/ASan检查有并发负载。所有测量保留，不把它们当作无干扰CPU测试或FIFO时限证明。总计仅含补值、方向和区域索引，不含GPU渲染、DDS/同步、首次原图复制或完整控制算法：

| 样本 | 方向索引均值ms | 区域索引均值ms | 含补值总均值ms | 总最大ms | 区域查询均值μs |
|---|---:|---:|---:|---:|---:|
| 初测首组 | 8.344080 | 0.303572 | 8.869820 | 9.082363 | 4.737 |
| 初测末组 | 8.757292 | 0.329991 | 9.328899 | 13.381629 | 4.735 |
| 最终首组 | 8.548858 | 0.304197 | 9.081360 | 9.359171 | 4.729 |
| 最终末组 | 8.547412 | 0.309697 | 9.085636 | 9.674161 | 5.003 |

区域查询测量混合完整图与内缩1像素的图内矩形；单像素最终均值约0.022～0.023μs，已接近计时开销，不能当严格延迟保证。当前查询远小于2ms，但新版本实际读取抖动/端到端数据年龄与FIFO在线表现尚未测量。

![索引建立与查询计时](../../../EGO4P0/docs/performance/depth_steps34_20261102_232509/direction_query_timing.png)

[计时PDF](../../../EGO4P0/docs/performance/depth_steps34_20261102_232509/direction_query_timing.pdf) · [最终建索引CSV](../../../EGO4P0/docs/performance/depth_steps34_20261102_232509/final_last/build_timings.csv) · [最终查询CSV](../../../EGO4P0/docs/performance/depth_steps34_20261102_232509/final_last/query_timings.csv) · [独立校验JSON](../../../EGO4P0/docs/performance/depth_steps34_20261102_232509/final_last/direction_query_validation.json)

## 输出文件、配置与验证边界

在线读取结束后的首末快照新增：

- `first/last_direction_offsets.bin`：uint32_le，格数+1，按β行、α列索引来源列表。
- `first/last_direction_sources.bin`：uint32_le，只有source_count个当前组来源码；从同组D_filled恢复Z。
- `first/last_direction_coverage.bin`：每角格一个uint8来源位，空格为0；这是角格来源信息，不是图像像素有效性掩码。
- `first/last_observation_index.json`：分辨率、网格大小、组身份、来源/无观测/每相机覆盖及几何排除统计。
- `first/last_region_tiles.csv`、`first/last_query_examples.csv`：每路摘要和带状态/覆盖/组身份的实际查询样本，无结果时Z列为空。

原始/补值图、b/d、K/T_BC继续保存。`frames.csv`增加方向/区域建索引耗时、来源数和空格数；`reads.csv`增加单像素与完整前相机区域查询探针，读取计时含这些查询，完整图探针确认0像素重扫。离线分析新增全部来源、格覆盖、摘要和首末查询校验；整个相机没有有限数值时仍保存合法空结果，统计最小/最大值为null，不伪造有限距离。

普通编排器与C++均支持`--azimuth-resolution-deg`、`--elevation-resolution-deg`，默认均1。已选一次性管理员入口`--mode optimized`继承该默认值，FIFO10＋200μs、RTX5070/cuda:0、500仿真Hz、0.02倍/每轮.2s不变：

```bash
cd /home/starry/isaac-data/EGO4P0
EGO4P0_BUILD_TESTING=ON bash scripts/build_isaac_ros_workspace.sh
ctest --test-dir docs/performance/runtime/build/new_controller --output-on-failure
# 新版在线入口，本轮未运行；默认1°×1°
sudo /usr/bin/python3 -I /home/starry/isaac-data/EGO4P0/scripts/run_reader_privileged_check.py --mode optimized
```

本轮三次4包Release构建成功（初次新测试缩进警告已修复，最终仅既有禁用Python字节编译警告）；最终CTest3/3通过。新增检查涵盖非归一化b与外参平移/四路旋转、接缝/极点、多相机同格、全空帧清理旧来源、无结果/部分值/边界区域、4000次与直接扫描的ROI对照、稳态零分配及1000次图像/索引共同所有权并发发布；原200000次三缓冲及第二步检查保留。ASan/UBSan通过，因已知工具ptrace限制关闭LSan，不声称检查了泄漏。

独立NumPy只用于离线验证，核对全部298795来源、机体系方向、相机覆盖、1200个摘要、25种导出查询；真实数据没有需要1e-11rad边界容差才能解释的不同单元分配。运行算法与查询全部为C++。未运行新Isaac/FIFO、导航、候选生成、轨迹碰撞判定或实机；旧FIFO短测仍属于旧源码。

[实现身份和全部检查](../../../EGO4P0/docs/performance/depth_steps34_20261102_232509/implementation_validation.json)。修改前源码/记录/实际读取ELF保存于`/home/starry/isaac-data/HISTORY/snapshots/EGO4P0_steps34_20261102_232509`；按文件回退并重建，保留新旧成功和失败，不覆盖共享历史总账。
