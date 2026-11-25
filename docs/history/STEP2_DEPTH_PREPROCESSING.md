# 第二步：光轴深度、异常像素补值与像素视线表

2026-11-02 22:57 +0800。C++实现已接入现有接收/三缓冲/独立2ms读取链路，4包Release构建、CTest2/2、真实Isaac存档图像逐像素验证通过。本轮采用已有数据离线验证，未新启动Isaac、未执行修改后的FIFO10在线测试；严格2ms及新帧完整性没有新增通过结论。

## 深度含义与补值规则

输入仍为`D_i(u,v)=Z_i(u,v)`，单位米、float32，表示相机光轴方向深度。当前仿真适配使用ROS2CameraHelper的`type=depth`；本机Isaac实现将其映射为`DistanceToImagePlaneSD`，与另一个`distance_to_camera`分支区分。仿真适配和C++标定校验保持`optical_axis_z_m`，未缩放深度或把Z转换成射线距离。

补值算法位于[depth_preprocess.hpp](../../src/new_controller/include/new_controller/depth_preprocess.hpp)：

- 仅有限且>0的原始像素有效；NaN、±Inf、±0和负数都属于异常。
- 有效像素逐位保持，不平滑、不截断有效深度。
- 对异常像素，只统计本图原始3×3邻域中有限且>0的深度，用double累加、求均值后写float32输出。
- 邻域裁剪到图像边界；四路独立处理，不跨相机取值。
- 原始数组与输出数组不重叠。计算只读原始数组，不读取刚补好的结果，因此不发生向外扩散。
- 无有效邻居时，保留原始异常值，包括NaN/Inf/零/负数的原始位模式；不将大块未知区域强制填成有限距离。

## 像素视线查询表

每路保存`b_i(u,v)=K_i^-1 [u,v,1]^T`和`d_i=b_i/||b_i||`。表使用float64，按`[camera][v][u]`行优先存储；u为列、v为行，像素坐标直接使用整数u、v，没有另加0.5。b和d均在各相机光学坐标系中，d为单位方向，表与图像深度值无关。

`PixelRayTables::configure()`在接收/控制线程启动前生成表。相同内参和尺寸再次配置直接复用，不分配内存；本节点运行期间固定标定，CameraInfo持续校验，发生不匹配时拒绝继续而不并发改写正在查询的表。通用配置函数支持完整3×3内参求逆，并在检查中覆盖非零skew；当前相机实际K为标准无畸变针孔形式。

四路320×240共307200个像素，每个像素保存b、d两个三维方向向量，表内存14745600字节（14.0625MiB）。不乘深度生成空间点，不生成三维点云或环境地图，也不把d变换成世界方向。Z仍是光轴深度，后续使用单位方向d时不能直接把Z当作射线距离。

## 接收与2ms读取的分工

```text
四路相机 → 按采集时间戳同步 → 复制原始D
                              → 四路独立原始3×3补值
                              → 完整raw+filled组发布至预分配三缓冲
                                            ↓
                               2ms线程借用最新完整组
                               复用补值结果和固定b/d视线表
```

补值每个新完整同步组只做一次，位于普通调度的C++接收线程、发布缓冲区之前；2ms循环读取原始/补值图像引用和已计算的视线表，不重复扫描、补值或生成表。读取期间两类图像属于同一受保护槽，接收端不能覆盖。相机未更新时复用最新完整组，未齐组仍不发布。

输入接口位于[depth_input.hpp](../../src/new_controller/include/new_controller/depth_input.hpp)，调用示例：

```cpp
#include "new_controller/depth_preprocess.hpp"
// input is a valid ControllerInputView borrowed for the current read cycle.
const auto& ray = input.pixel_rays->at(camera, u, v);  // validates camera/u/v
const size_t pixel = v * input.width + u;
const float raw_z = input.depth->D[camera][pixel];
const float filled_z = input.depth->D_filled[camera][pixel];
const auto& b = ray.b;
const auto& d = ray.d;
```

图像/状态引用只在当前读取所有权期间使用，不跨下一次`latest()`保存。视线表在整个运行期间只读。图像缓冲区启动时分配；3个raw+filled槽、固定首组快照与可选复制槽约11.72MiB，加视线表约25.78MiB，不含DDS消息和计时元数据。没有完整图像历史队列。日志和首末样本文件在读取结束后写入。

## 本轮真实数据离线验证与耗时

数据来源为之前真实Isaac会话`privileged_20261102_181620/fifo10_200us`的first/last原始深度与实际相机参数。本轮没有生成或伪造新仿真帧。独立NumPy离线参考对所有像素用原始图移位累加，逐位核对补值图；验证原有效值及未解异常值逐位保持，并逐像素核对b/d、单位范数、相机顺序和二维索引。

两张四路样本的异常分布相同；每组307200像素中原异常11010个，补齐2605个、按规则保留8405个：

| 相机 | 原始异常 | 有效邻居均值补齐 | 无有效邻居，原值保留 |
|---|---:|---:|---:|
| Front | 836 | 432 | 404 |
| Left | 3516 | 1142 | 2374 |
| Back | 5902 | 701 | 5201 |
| Right | 756 | 330 | 426 |

性能测量为CPU5、SCHED_OTHER/优先级0，50次预热后1000次重复处理同一缓存样本；包括四路原始→补值复制、异常识别、均值和计时开销，不含DDS/仿真渲染或首次原始消息复制。不是1000组新相机图，也不是FIFO定时读取间隔测试。初版及最终两组测量均保留，不只挑最小值：

| 测量 | 四路补值均值ms | P99 ms | 最大ms | 视线表初始化ms（循环外） |
|---|---:|---:|---:|---:|
| 初版首组 | 0.184638 | 0.237508 | 0.289151 | 5.169300 |
| 初版末组 | 0.179870 | 0.212883 | 0.238654 | 3.742502 |
| 最终首组 | 0.175467 | 0.217967 | 0.243776 | 4.964685 |
| 最终末组 | 0.175062 | 0.197649 | 0.232143 | 3.984589 |

最终两组四路补值均值约0.175ms；视线表只生成一次，约4～5ms，未放进2ms循环。在线接收负载、数据年龄、相机未齐组及FIFO读取抖动仍需用新程序另测。

![离线补值耗时曲线](../../../EGO4P0/docs/performance/depth_step2_20261102_223955/preprocess_timing.png)

[曲线PDF](../../../EGO4P0/docs/performance/depth_step2_20261102_223955/preprocess_timing.pdf) · [最终首组原始计时CSV](../../../EGO4P0/docs/performance/depth_step2_20261102_223955/final_first/timings.csv) · [最终末组原始计时CSV](../../../EGO4P0/docs/performance/depth_step2_20261102_223955/final_last/timings.csv) · [逐像素校验](../../../EGO4P0/docs/performance/depth_step2_20261102_223955/final_last/preprocess_validation.json) · [构建与验证记录](../../../EGO4P0/docs/performance/depth_step2_20261102_223955/implementation_validation.json)

## 运行输出与复现

新读取运行继续保存原始`first_depth.bin`/`last_depth.bin`，增加`first_filled_depth.bin`/`last_filled_depth.bin`（float32，shape=4×height×width），以及`pixel_rays_b.bin`/`pixel_rays_d.bin`（float64，shape=4×height×width×3）。NPZ输入快照保留原D1～D4并新增D1_filled～D4_filled。

`frames.csv`新增四路补值总耗时、原始复制耗时、每路invalid/filled/unresolved计数与各路耗时。`reads.csv`保留实际2ms调度与读取耗时，增加补值图像及视线探针，避免仅测空引用。`reader_meta.json`记录预处理位置、视线表生成次数/耗时/字节与格式。离线分析新增首末补值及完整视线表校验，不把读取引用耗时当作整张图的补值耗时。构建身份现在包含预处理头文件与CMake配置，执行时继续核对实际源码及ELF。

```bash
cd /home/starry/isaac-data/EGO4P0
EGO4P0_BUILD_TESTING=ON bash scripts/build_isaac_ros_workspace.sh
ctest --test-dir docs/performance/runtime/build/new_controller --output-on-failure
# 选定FIFO10＋200μs方案，0.02倍速、每轮0.2s仿真；本轮尚未执行这条新版本在线命令
sudo /usr/bin/python3 -I /home/starry/isaac-data/EGO4P0/scripts/run_reader_privileged_check.py --mode optimized
```

离线参考脚本`verify_depth_preprocessing.py`仅用于结束后的验证；接收、补值、查询表与读取算法全部为C++。C++检查涵盖3×3均值、NaN/Inf/零/负数、边缘/单行/单列、不扩散、四路独立、溢出安全、有效/未解像素位保持、非零skew求逆、查询表复用与1000次零稳态分配。旧三缓冲200000次并发发布与同步检查仍通过。

本轮两次4包Release构建成功，最终CTest2/2通过；ASan/UBSan在关闭LSan后通过。初次默认Sanitizer运行在断言通过后因工具ptrace环境使LeakSanitizer报错退出1，保留日志，不声称完成泄漏检查。初次绘图因用户NumPy2与系统Matplotlib不兼容失败，改为`PYTHONNOUSERSITE=1 /usr/bin/python3`后成功，无依赖版本修改。未运行新的Isaac/FIFO、旧231项全回归、导航或实机；旧FIFO成绩仍只属于修改前版本。

修改前源码/文档/实际读取ELF保存在`/home/starry/isaac-data/HISTORY/snapshots/EGO4P0_step2_20261102_223955`。回滚按文件恢复并重建，保留本轮成功、失败和旧数据，不覆盖共享历史总账。新版本仍没有运动控制或避障输出。
