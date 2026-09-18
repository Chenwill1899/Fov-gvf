# MISSANDKEYBOARD：手柄意图 + 深度避障

本目录把 `KEYBOARD/` 中已验证的北通 A2P3A Mode 2 映射接入 USER 的完整
FOV-GVF 深度避障链。`KEYBOARD/` 仍是无 ROS、无避障的纯手柄参考，不在这里复制或
修改。

## 运行

首次运行或源码更新后先构建：

```bash
cd /home/starry/isaac-data/EGO1P0
bash scripts/build_isaac_ros_workspace.sh
```

连接北通手柄后：

```bash
cd /home/starry/isaac-data/EGO1P0/MISSANDKEYBOARD
bash run_joystick_avoidance.sh
```

组合入口默认使用 4 倍 Cloud 障碍场景。临时使用平地可执行：

```bash
FOV_GVF_SCENE_MODE=flat bash run_joystick_avoidance.sh
```

## Mode 2 与控制链

- 左摇杆上下：Throttle，升降；
- 左摇杆左右：Yaw，只改变机头方向；
- 右摇杆上下：Pitch，沿当前机头前进/后退；
- 右摇杆左右：Roll，沿当前机头左移/右移。

右摇杆和油门先转换成 world 坐标的 `/human_intent`，再由 C++ 深度角域 FOV-GVF
根据期望XY方向选择前/左/后/右深度视角并修正，输出 `/position_cmd -> /sim/cmd_vel`。手柄不会直接写无人机的
平移位置；Isaac 末端还保留 ESDF/AABB 碰撞保护。Yaw 不属于现有 FOV-GVF 的平移
避障量，由手柄独立控制相机朝向。

启动时四个摇杆必须在死区内连续回中 0.5 秒才解锁。摇杆回中立即悬停；USB 断连
会同时清零平移和偏航。默认设备与映射为：

```text
/dev/input/by-id/usb-BEITONG_BEITONG_A2P3A_BFM_DONGLE-joystick
Yaw=0, Throttle=1, Roll=2, Pitch=3
```

可继续使用 `KEYBOARD/README.md` 中的 `JOYSTICK_DEVICE`、`JOYSTICK_AXIS_*`、
`JOYSTICK_SIGN_*`、速度、死区和回中保持时间环境变量。视口仍可按 `1`、`3` 或
`V` 切换第一/第三人称。

## 职责边界

该入口启用 ROS 2、四路水平深度相机、FOV-GVF、RViz、在线观测地图、性能记录和最终
碰撞保护。它是水平360°人工局部避障，不是全局规划或上下三维绕障；旋转机头会同时
改变四台深度相机的观察方向。
