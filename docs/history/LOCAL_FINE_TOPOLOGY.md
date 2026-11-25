2026-11-16 12:47 +0800 补充：在本页第一步基础上，已采用严格输入粗图缓存，省去相同输入的重复构图/证书扫描，原拓扑和细场语义保持。详见[第二步实测与安装](../../../EGO4P3/docs/performance/coarse_cache_20261116/coarse_cache_fifo_20261116_123841_284447/REVIEW.md)。

2026-11-16 12:26 +0800 — **局部细场第一步FIFO5已复核并采用。** 参考几何样本冷算均2.171→1.690ms（-22.15%）、最大5.801→4.240ms、>5ms为2→0；复用均值略增。12,000对功能结果一致，生产安装已更新且运动关闭。有限离线样本不代表长期5ms或飞行验收。 [本次复核](../../../EGO4P3/docs/performance/local_fine_20261116/local_fine_fifo_20261116_121555_211145/REVIEW.md)。下方待测状态保留为测量前记录。

# EGO4P3 局部细场：省去未使用的粗势数值求解

2026-11-16。本轮基线是已经完成二维内禀计算优化的 P3；不是最初的 P2，也不是上轮尚未优化的算法。逐步优化的第一项只处理默认局部细场中的冗余粗求解，验证与耗时状态以 [本轮报告](../../../EGO4P3/docs/performance/local_fine_20261116/REPORT.md) 为准。

## 为什么可以分离

默认 `fine_neumann_roi=true` 时，细域骨架来自细网格 BFS，再扩三层邻居。细层只继承自适应粗域的 owner 和 Exit 标签，重新设置局部 Low=0、Exit=1；侧面采用零通量。粗势值没有进入细矩阵、细边界或方向曲线。原实现仍强制进行一次粗数值求解，并以求解通过、粗场缓存命中作为后续检查条件。

本轮保留粗域构造、完整图结构、出口标签和细域范围，把默认粗层的数值检查改为明确的拓扑检查。细层仍使用原 LDLT、残差与势值范围检查，后续方向曲线、三维积分、原始深度管道与执行约束保持。

## 拓扑检查与缓存

`CoarseTopologyValidator` 验证 component/active/owner 的完整对应关系、节点代表与边界标签、CSR 边界、正有限权重、有限正度数、反向边及对称性、整个图的连通性，以及 Low/Exit 同时存在。缓存精确比较 owner、representative、boundary、offset、neighbor 和 weight；不使用仅哈希相等授予复用。缓存命中前仍核对本次 component 和 active 的覆盖关系。检查失败使拓扑缓存失效，几何、坐标、配置变化及显式冷算沿用失效规则。

这证明需要的图与边界结构成立，并不声称已经求得粗调和势或通过粗数值残差检查。它不能保证旧粗求解器在任意有限精度条件下一定求解成功；当前设计将执行所需的数值条件落在实际使用的细场上。功能对照只能说明覆盖样本中的一致性。

## 状态与诊断

- 默认粗层成功状态为 `TopologyReady`，不是数值场的 `Valid`。
- 未求解的粗势与残差使用 NaN；JSON 残差导出为 `null`，CSV 使用 `nan`。不填零冒充已求解。
- 默认粗层 `solve_ns`、`solve_cpu_ns`、迭代次数和数值后端统计为零；拓扑检查计入独立 `coarse_topology_ns`。
- 默认粗层的 `reused` 表示相同拓扑证据复用；细层 `reused` 仍表示数值场复用。
- `fine_neumann_roi=false` 的粗势边界耦合分支仍进行完整粗数值求解。
- `force_coarse_solve=true` 为同源码诊断对照保留原默认数值路径，不通过放宽残差实现一致。

## 运行与证据边界

```bash
/usr/bin/python3 -I /home/starry/isaac-data/EGO4P3/scripts/build_local_fine.py --tests
/usr/bin/python3 -I /home/starry/isaac-data/EGO4P3/scripts/run_local_fine_fifo.py --correctness --rounds 1 --repeats 2
sudo /usr/bin/python3 -I /home/starry/isaac-data/EGO4P3/scripts/run_local_fine_fifo.py
```

新构建和回放写入独立的 `docs/performance/local_fine_20261116`，不覆盖上轮 planar 结果和生产安装目录。两版使用同一测量驱动；冻结基线不改，另建只加入对称 CPU 计时的测量副本并记录补丁。驱动记录绝对释放、开始、结束、下一释放时间，粗细层耗时及协调线程 CPU 时间；协调 CPU 不包含四路相机线程 CPU。

真实存档缺少近裁面证据的事实保持。新增明确标注的合成标定净空用例用于补充正例，不能当作真实相机或飞行验收。正式性能仍要求五线程 FIFO5/CPU0～4 实际读回；普通调度只做正确性检查。5 ms、12 m/s、4 m、0.30 m 包络及默认运动输出关闭保持。
