# BEITONG Mode 2 独立手柄控制测试

目录名和启动脚本名为兼容原入口而保留。当前程序读取 Linux USB joystick，按
多旋翼常用的 Mode 2 方式直接控制无人机，不再用键盘控制飞行，也不启动或依赖：

- ROS 2；
- FOV-GVF 避障控制器；
- 深度图控制链；
- ESDF 或障碍碰撞预测；
- RViz 和在线地图。

运行：

```bash
cd /home/starry/isaac-data/EGO1P0/KEYBOARD
bash run_joystick_only.sh
```

默认读取稳定设备路径
`/dev/input/by-id/usb-BEITONG_BEITONG_A2P3A_BFM_DONGLE-joystick`。当前机器已识别到
`BEITONG BEITONG A2P3A BFM DONGLE`（USB `20bc:511c`），8轴、16按钮。终端会打印设备
信息、四通道值和控制是否解锁。

## Mode 2 控制

- 左摇杆上下（Throttle）：上升/下降；中位为零垂直速度
- 左摇杆左右（Yaw）：机头左转/右转
- 右摇杆上下（Pitch）：沿当前机头方向前进/后退
- 右摇杆左右（Roll）：沿当前机身方向左移/右移

内核轴码表确认北通接收器的轴0/1是左摇杆X/Y，轴2/3是右摇杆X/Y，轴4/5是
Gas/Brake扳机，轴6/7是方向键。因此默认使用轴0 Yaw、轴1 Throttle、轴2 Roll、
轴3 Pitch；扳机和方向键不参与飞行控制。
程序会修正 Linux 竖轴方向。水平最大速度 `2.0 m/s`、垂直最大速度 `1.5 m/s`、
最大偏航角速度 `75 deg/s`。启动后需将四个摇杆保持中位 `0.5 s` 才会解锁；拔掉USB
后立即停止运动。

可调参数示例：

```bash
JOYSTICK_HORIZONTAL_SPEED=1.0 \
JOYSTICK_VERTICAL_SPEED=0.8 \
JOYSTICK_DEADZONE=0.10 \
bash run_joystick_only.sh
```

如果设备节点或通道枚举不同，可使用 `JOYSTICK_DEVICE`、`JOYSTICK_AXIS_ROLL`、
`JOYSTICK_AXIS_PITCH`、`JOYSTICK_AXIS_THROTTLE`、`JOYSTICK_AXIS_YAW` 覆盖。每个通道
还可通过对应的 `JOYSTICK_SIGN_*=-1|1` 反向。

默认持续运行到关闭Isaac窗口或按 `Ctrl-C`。自动限时观察可使用：

```bash
JOYSTICK_TEST_TIMEOUT=5 bash run_joystick_only.sh
```

唯一保留的空间限制是 `z>=0.25 m` 的平地边界，防止持续下降后无人机穿过地面。
它不进行障碍检测、路径修正或避障限速。完整 USER 控制链仍使用上级目录中的
`scripts/run_isaac_fov_gvf_navigation.sh`，与本测试入口互不调用。

## 观察辅助

独立场景运行时会把太阳光强度从原来的 `800` 降为 `2.5`，并关闭自动曝光，避免
无人机高光区域过曝。出生点下方有固定的深色中心台、红绿坐标十字和四个方向块：
橙色为 `+X`、蓝色为 `-X`、
绿色为 `+Y`、品红色为 `-Y`。这些参照物只用于观察，没有碰撞，也不参与控制。

需要自动保存第三人称视图截图时，可指定：

```bash
JOYSTICK_TEST_TIMEOUT=4 \
JOYSTICK_SCREENSHOT=/tmp/joystick_view.png \
bash run_joystick_only.sh
```
