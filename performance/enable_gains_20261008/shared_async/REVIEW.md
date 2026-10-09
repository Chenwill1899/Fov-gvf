# 共享异步发布修复与验证

本次只修 `depth_angular_controller_node.cpp` 的本地共享发布时间选择：由第一有效相机时间改为全部当前有效本地观测时间的最大值。每条track仍使用原观测时间，原0.5s时效/+20ms未来检查、来源及序列规则不变；没有转发peer占据，没有修改默认开关、CMake或动态证据策略。

新增 `tools/ros_shared_async_probe.py`，通过可控ROS时钟、两个异步合成相机及固定真实ROS控制器验证。主构建由主线程执行，本代理没有独立build或Isaac。

## 冻结旧版与当前安装版

相同输入：front固定100.00s，left先100.12s后100.20s；此时front尚未超过0.3s有效期。

- `before.json`：旧版这两阶段均无新本地共享包，直接复现第一相机钳制。原时间保留、不转发检查的合取也因此为false，不能解释成旧版改写时间或转发peer。
- `after.json`：新版分别发布100.12/100.20s包，新侧向命中的原时间被保留；仍携带100.00s旧track而未刷新其时间。全部10项检查通过。
- 注入的remote_only占据确实被接收（诊断接受计数≥1），但没有出现在本地发布包。
- 100.80s时过期命中消失，发布为空；100.90s本地侧向仍可发布，101.10s未来front未被摄入/发布；所有发布track满足原时效关系，序列严格递增、world frame不变。
- 两次子控制器均exit0且已清理，旧探针整体exit1为预期回归复现，新探针exit0。完整包、脚本/控制器SHA、参数和controller日志分别保存。

运行命令（先source ROS和本项目install；输出目录必须全新）：

```bash
PYTHONNOUSERSITE=1 python3 tools/ros_shared_async_probe.py --controller /absolute/controller --output /new/result.json --domain-id 78
```

此次旧/新分别使用domain78/79。没有使用墙钟计时收益作为验收。

## 原有ROS正负复验

- `shared_ros_control.json`（domain80，exit0）：危险阶段独立版平均0.500m/s，共享版101样本全0；清除后共享版恢复0.493117m/s。无危险阶段两版均可通行。
- `shared_ros_all_features.json`（domain81，exit0）：两机器人各发布93包、各接受93包、各拒绝93个错误frame包。此脚本的 `--all-features` 明确包含③⑤，只是已有兼容性负探针，不能作为拟默认①②④⑥配置的整机无退化证据。
- 所有临时controller日志已原样复制到本目录，映射和SHA见 `validation_summary.json`。最终进程检查未发现控制器或共享探针残留。

## 只读审查发现，未扩大修改

- ②开启会禁止后续永久行经/认证体积，历史TTL≤0.5s、采样间隔≤0.1s、容量至少32、历史不确定性≥0.35；初始化seed仍无TTL。原静态launch参数dump可能仍显示静态设置，因为实际覆盖发生在PaperGuidance构造内部。⑥单独开启只开启本地tracker和peer约束，不触发②的历史覆盖。
- ②旧解析闭环确有代价：穿越让行进展3.422→2.000m、远离障碍5→4.888m；空场5→5m。属于安全响应/进展代价，不能写成所有开启指标均不退化；没有动态Isaac总体结论。
- 默认source ID为ego1p5，同ID包在比较session前被忽略。真实多机必须唯一FOV_GVF_ROBOT_ID、共享坐标/时钟，私有odom/depth/intent/command话题隔离。当前默认单机没有peer，此项不改变运行结构。
- 每视角overloaded状态没有TTL。一视角非法或超过1024格后若不恢复正常帧，即使其他视角继续更新，也会持续失败关闭；⑥会发布此overloaded状态。未修改这一保护，也未添加未验证的自动释放。
- 共享包限制8来源、每来源4096条、0.5s TTL；第9有效危险触发暂时停车，错误frame/非法包不触发容量锁。共享只提供占据约束，不提供自动配准或远端自由空间认证。

必须保留的现有核心检查为dynamic_obstacles_check、dynamic_closed_loop_check、shared_obstacles_check、six_axis_replay_check及其goal版本。可用主线程构建产物执行：

```bash
ctest --test-dir /tmp/fov_gvf_ego1p5_isaac_build/pc_gvf --output-on-failure -R '^(dynamic_obstacles_check|dynamic_closed_loop_check|shared_obstacles_check|six_axis_replay_check(_goal)?)$'
```

本次代理只运行上列ROS探针及脚本语法检查；统一核心回归由主线程记录。本文件不是默认启用或动态/多机飞行的验收报告。
