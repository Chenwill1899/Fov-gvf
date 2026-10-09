# EGO1P5 第二轮⑥共享占据优化

本阶段比较原版P5共享占据实现与本轮改进，原版源来自本轮`../baseline/sources.tar.gz`。共享地图仍只传递同一world坐标系的占据与运动证据，不传播自由空间，不替代本机完整运动认证，也不假定自动完成多机坐标/时钟配准。

## 方案和实现

1. **容量保护**：旧版已经接收8个源后，拒收第9个有效危险源，但随后的`segmentSafe`仍可能授权穿越该源报告的危险。新版在完整验证消息后遇到容量不足，只要有效消息含占据或overloaded，就保持容量保护至该消息原stamp+.5s。拒绝错误frame/非法消息不会触发锁；无危险且未过载的空消息不触发锁；不会刷新或延长原始TTL。内存上限仍8源、每源4096轨迹。新增锁状态通过root负责的M版快照保存；旧快照默认无锁。
2. **保守宽相**：每次接纳/恢复源消息时，缓存占据AABB、每轴最大绝对速度、最大半径与原始时间范围。运动查询时用最早stamp、全部原半径、最坏3m/s速度误差和.5m/s²加速度向外扩张，只跳过确定与完整运动段分离的源。相交时调用原精确相对运动证书。缓存随原消息重建，不需额外快照字段；不合并/删除任何原轨迹。

变更：`shared_obstacles.hpp/.cpp`、`shared_obstacles_check.cpp`；动态模块负责的共用证书有限数校验同时生效。root另行记录`paper_replay.cpp`及快照测试。

## 同输入消融

32个有效第9源危险场景：旧版错误放行**32/32**，新版**0/32**。32个错误frame负例均没有使原本可用地图停机；32个过期容量保护均在1.501s正常释放；32个正常接收近障碍均被拒绝。

固定8个源，各32/256/1024/4096条轨迹，先融合远离查询路径的占据，再重复3000次同样查询，每组5次。两版调用同一个`refinement_probe.cpp`、同编译参数`-O3 -DNDEBUG -std=c++14`。所有决定哈希相等；每组额外1500条固定种子随机路径，全部30000条宽相/精确决定一致。大图远障query阶段原版均值494.8016ms、新版.1946ms；此数据**只计查询，不含merge缓存预计算，不能推广成共享整链路快2500倍**。全部规模及5次样本见`summary.json`、原始CSV。

数值贴边回归包含epoch=0/1e9/2e9、多个预测时长、胶囊边界两侧，以及未来20ms舍入边界，共63条；另外2000条固定种子随机路径，均与原精确函数一致。完整本地未知仍拒绝、过期/乱序/错frame等已有回归保留。主CTest和共享锁快照回归由root统一构建运行，本文不提前认定通过。

## 审查发现与修复

审查代理发现宽相第一版先计算`now+end-earliest`，在Unix大时间戳下可能比精确函数`(now-stamp)+end`少算约1e-7s，贴边危险可被误跳过。另发现`latest>now+.02`与精确`now-stamp<-.02`在20ms边界浮点不等价。均已改为与精确函数相同运算顺序，并加入生产check回归；没有以加大经验阈值掩盖问题。

首次计时保留为`optimized_initial.csv/.stderr`，时间顺序修复、未来时间边界修复之间的试验保留为`optimized_before_future_edge_fix.csv/.stderr`；最终源哈希记录于`source_sha256.json`。`safety_regression.log`对应独立实际生产模块边界回归，通过63+2000条；可执行探针源保留。

这些是解析模块实验，未做新多机ROS闭环、Isaac多机飞行或真机。缓存增加一次线性merge扫描和每源少量派生状态，该成本未单独计时；持续近障碍/包络相交仍走精确扫描，不保证任意地图都有加速。第9个未知危险源导致暂时停机是明确的保守代价。

## 复现

基线解压到独立目录，新旧都使用本目录同一个探针。令`SOURCE`分别为解压根目录与当前EGO1P5根目录，输出到新目录：

```bash
c++ -O3 -DNDEBUG -std=c++14 -I/usr/include/eigen3 -I"$SOURCE/src/pc_gvf/include" \
  refinement_probe.cpp "$SOURCE/src/pc_gvf/src/depth_angular_core.cpp" \
  "$SOURCE/src/pc_gvf/src/dynamic_obstacles.cpp" "$SOURCE/src/pc_gvf/src/shared_obstacles.cpp" \
  -o /tmp/shared_probe
/tmp/shared_probe
```
