# 手柄与目标运行参数

```bash
cd /home/starry/isaac-data/EGO2P6
bash scripts/run_manual.sh
```

右杆前后左右，左杆升降/偏航；四轴回中0.5秒解锁，关闭Isaac窗口或Ctrl+C退出。默认同时显示Isaac和RViz。

目标意图沿用手柄避障策略：

```bash
EGO2P6_GOAL_ROUTE=r07 EGO2P6_GOAL_HEADLESS=0 EGO2P6_GOAL_RVIZ=true bash scripts/run_goal.sh "r07_$(date +%Y%m%d_%H%M%S)"
```

路线r01至r10及route300。手柄策略use_fixed_goal=false，不能仅凭可执行文件名判断策略。保存参数脚本scripts/manual_profile.env，路线src/pc_gvf/config，安装docs/performance/runtime/install。参数10m/s请求、垂向1、depth20/H.5/cache.3/.75/history16/1s/a16/tau.10/延迟.20，完整值见EGO2P6_VERSION.md。

本机Humble、ROS域42、共用导航锁和单发布者检查保持，默认不绑核；可显式设置EGO2P6_CONTROLLER_CPU_AFFINITY。大型二进制诊断快照默认关，EGO2P6_RECORD_REPLAYS=1开启。输出和缓存位于docs/performance，旧目录入口已合并，不保留重复兼容副本。此目录重排未修改算法数值或服务器。
