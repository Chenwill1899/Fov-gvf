# EGO4P5 · 四相机球面自动巡航

EGO4P5 在 Isaac Sim 中沿给定航线飞行，支持 R05 Cloud 和四门机库，目标巡航速度为 12 m/s。默认策略 `directional` 用四路深度按需求解球面航向，再用当拍原始深度检查实际速度方向上的整条制动通道。未知区域不能提供通行证据。[当前算法与验证范围](PROJECT_STATUS.md) · [数学推导与源码对照](SPHERICAL_NAVIGATION.md#8-方向深度主版本)

## 启动和重建

在本机普通用户终端执行。前两条预检会校验发布清单、控制器、动态库和场景文件。最后一条启动 R05，将运行记录和渲染缓存写入 P5 的状态目录。

```bash
cd /home/starry/isaac-data/EGO4P5
/usr/bin/python3 -IB scripts/run_auto_cruise.py --scenario r05 --check
/usr/bin/python3 -IB scripts/run_auto_cruise.py --scenario hangar --check
/usr/bin/python3 -IB scripts/run_auto_cruise.py --scenario r05 --output-root "${XDG_STATE_HOME:-$HOME/.local/state}/ego4p5"
```

四门机库使用 `--scenario hangar`，无窗口运行加 `--headless`。正式仿真使用同工作区的 Isaac Sim 和五个 FIFO5 算法线程，启动器按需请求本机 sudo 认证。省略 `--output-root` 时沿用源发布的 `ego4p4` 状态目录。源码重建命令为 `/usr/bin/python3 -IB scripts/build_auto_cruise.py`。

## 从哪里读代码

| 要了解的部分 | 入口 |
| --- | --- |
| 一拍内如何选择航向、限幅和制动 | [控制器](../src/auto_cruise/controller.cpp) · [方向选择](../src/auto_cruise/directional_guidance.hpp) · [原图净空](../src/auto_cruise/image_depth_clearance.hpp) |
| 如何从深度构建方向域、求调和场、纠正航线横偏 | [导航流水线](../src/new_controller/ldlt/include/new_controller/spherical_navigation.hpp) · [球面势场](../src/new_controller/ldlt/include/new_controller/spherical_field.hpp) · [位置流盒](../src/new_controller/ldlt/include/new_controller/position_flowbox.hpp) |
| 场景、起终点、航线和仿真轨迹的独立复核 | [双场景配置](../src/auto_cruise/scenarios.json) · [场景资产](../scenes) · [仿真接口](../src/auto_cruise/simulation.py) · [轨迹审计](../src/auto_cruise/audit.py) |
| 构建、运行参数与回归 | [启动和构建脚本](../scripts) · [控制参数](../src/new_controller/config) · [球面回归与夹具](../src/auto_cruise/tests) · [ROS 与测试构建](../src/new_controller/CMakeLists.txt) |
| 早期实现与实验 | [四路输入](history/STEP1_DEPTH_INPUT.md) · [深度预处理](history/STEP2_DEPTH_PREPROCESSING.md) · [局部细场](history/LOCAL_FINE_TOPOLOGY.md) · [滚动跟踪](../src/new_controller/ldlt/include/new_controller/rolling_navigation.hpp) · [R05 模型](history/R05_MODEL_SIMULATION.md) |
| 技术资料与维护记录 | [历史说明与讲义](history) · [公式与概念用途版](history/公式与概念用途版.tex) · [历史实验工具及接口](performance) · [工作日志及恢复入口](WORK_LOG.md) |

早期文档中的命令、参数和成绩对应各自的版本。P5 的双场景飞行证据可由项目状态页追溯至源发布。历史对照实验的原始运行数据仍在对应项目或工作区归档中。
