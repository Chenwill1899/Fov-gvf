# EGO1P1 深度避障开发主线

## 构建与运行

```bash
cd /home/starry/isaac-data/EGO1P1
bash scripts/build_isaac_ros_workspace.sh
bash scripts/run_isaac_fov_gvf_navigation.sh
```

需要本机 `/opt/ros/humble`、`/home/starry/isaac-data/isaacsim`、可用 NVIDIA 图形环境及北通 A2P3A BFM/XInput 手柄。默认打开 4 倍 Cloud、Isaac Sim 和 RViz；手柄四轴回中 0.5 秒后解锁。关闭窗口或 Ctrl-C 退出。

无手柄时显式使用键盘：

```bash
ISAAC_MANUAL_INPUT_MODE=keyboard bash scripts/run_isaac_fov_gvf_navigation.sh
```

键盘 W/S、A/D、R/F 为世界坐标前后、左右、升降；Space 停止，1/3/V 切换视角。平地测试用 `FOV_GVF_SCENE_MODE=flat`，关闭 RViz 用 `FOV_GVF_RVIZ=false`，限时用 `ISAAC_MANUAL_TIMEOUT=5`。

## 保留目录

| 路径 | 用途 |
|---|---|
| scripts/*.sh | 仅两个主入口：构建、运行 |
| scripts/isaac/ | Isaac 运行器、手柄输入、XY/Z 控制数学 |
| scripts/lib/ | BFM/XInput 设备识别 |
| src/pc_gvf/ | C++ 避障、launch、测试及 Python 数值/合成验收基准 |
| src/pc_gvf_msgs/ | PositionCommand 消息 |
| src/pc_gvf_platforms/ | 命令桥、RViz/在线地图和相关测试工具 |
| scenes/ | Cloud USD+occupancy.bin、平地导航 USD |
| tools/ | 算法基线生成与 ROS 回归探针，不是日常运行入口 |
| performance/ | 逐次运行性能记录（含继承历史） |

构建产物使用 `/tmp/fov_gvf_ego1p1_isaac_{build,install,log}`，无需项目内 build/install/log。修改源码后重新构建。算法参数集中于 `src/pc_gvf/launch/isaac_cloud_navigation.launch.py`。

当前是四相机水平 360°局部避障，Z 独立控制；历史地图只显示，ESDF 仅做仿真末端保护。与其他副本共用 ROS 域42/单实例锁，不要同时运行。

版本和整理归档见 [EGO1P1_VERSION.md](EGO1P1_VERSION.md)，实际变更与验证见 [WORK_LOG.md](WORK_LOG.md)。
