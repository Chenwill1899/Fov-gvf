# 中间实验记录（不是最终验收）

这些原始日志保留本轮探索、失败和修正，不能混合为一次通过结果。最终选定版本的证据见父目录及项目 PAPER_ACCEPTANCE.md。

- 最初无几何初始化时四水平相机不能认证自身包络，拒绝运动；旧验收报告单独保留。
- 初次起点证书读到 OmniGraph 零位姿而拒绝，改为等待实际起始位姿后一次验证。
- 多次几何/网格/前视/历史配置实验仅前进约1.5–2.5m后停止，不算导航通过。
- `paper-verified-native.log`：全分辨率几何查询开销超过300ms，未解决停滞；撤销该方式。原始像素障碍命中仍保留作证据冲突检查。
- `paper-verified-neighborhood.log`：五球启动邻域试验仍停滞；最终恢复一个固定半径2.5m起点球。
- `paper-verified-exact-hits.log`：期限监控和匹配制动前，最大求解约3.109s、外部碰撞保护介入142次，明确失败。
- `paper-verified-build18.log`：作用域编译失败。紧随的 `paper-verified-direction-chart.log` 实际运行旧二进制，不能用于该改动验收。build19才修正；后续批次用set -e。
- `paper-verified-ctest.log`：两项Python收集遭用户NumPy ABI冲突；隔离用户依赖后通过。`paper-verified-platform.log` 未source工作区而导入失败，sourced日志修正。
- 深度无返回检查初期采用错误体素对齐/整数像素中心，分别出现95/22个伪冲突；按实际USD体素中心和Isaac半像素中心重新做精确DDA，最终19,163条+Inf射线在10m内零冲突。没有过滤这些像素来凑结果。
- `paper-verified-fxaa.log`、`paper-verified-unjittered.log`：抗锯齿实验未解决主要问题，最终保持TAA默认。
- build31对应 `paper-verified-parallel-evidence.log` 和父目录 `paper-completion-isaac-repeat.log`：两次前进16.42/16.68m、零外部保护介入，但约30Hz且存在计算期限拒绝。
- build32进一步并行方向认证，使用各线程独立的当帧正证书缓存；父目录paper-parallel-isaac.log前进16.55m、零保护介入，40.97Hz，仍未达到每20ms一次更新。
- 最终build33只对球证据冲突检查作等价循环/平方距离优化，并补齐OpenMP导出依赖；最终验收另列。

曾提出通过ROS消息传递“自由空间证书”的设计，被自动审批指出可伪造来源，未创建该消息。最终由控制节点直接读取配对USD/occupancy并核验已知SHA-256和球内几何，不采信消息中的认证字符串。
