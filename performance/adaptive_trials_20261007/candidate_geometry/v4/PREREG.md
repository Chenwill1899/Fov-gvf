# v4 固定段几何缓存预注册

本轮最后一个独立几何候选。不使用v1/v2/v3任何AABB证据筛选。唯一改动：根球与中心快速检查失败并进入原八叉树后，若tube数>8，在该次envelopeKnownImpl局部容器中按原逆序计算每条段的delta=b-a和squaredNorm；后续查询使用同一差向量/平方长度，仍保留原dot/除法/clamp/减法/norm、半径比较、球和双球联合证明、证据顺序、level7与1024预算。异常元数据回旧tubeContains。

候选仅保存在本目录，冻结基线是../.. /baseline（实际路径由replay_ablation.py解析），链接冻结静态库，生产不改。先独立编译和数百万原始/缓存查询差分，包含首次接受证据顺序、阈值贴边、零长、1e-20长度平方附近、NaN/inf/大坐标。任何差异即拒绝，不以近似容差换取通过。

探索固定37输入（16卡停+21继承）、5批，逐帧AB/BA/AB/BA/AB。每批37帧P95为独立重复单位，5批P95均值与pooled P95均须下降至少10%，其他正式replay policy门槛也适用。仅探索通过时再跑全新5批确认（BA/AB/BA/AB/BA）。二进制和fixtures SHA256运行前后保持相同；全部原始输出逐字节相等。未达标立即终止并保留失败数据，不再增试。

本文件与外部assessment/geometry_v4_registration.json均在首次基准前写入。
