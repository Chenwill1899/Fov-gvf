# Fov-GVF for ROS 2

> **EGO1P0 当前运行入口：**本文件下方保留上游ROS 2说明；本机EGO1P0的实际
> Humble + Isaac Sim + 北通手柄避障功能、平台、参数和运行命令，以
> [EGO1P0_VERSION.md](EGO1P0_VERSION.md) 为准。

ROS 2 Jazzy workspace for depth/FOV-guided vector-field navigation. The
repository is self-contained; it does not require files from the old ROS 1
workspace.

## Build and test

```bash
./build_ros2.sh
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --base-paths src --build-base build --install-base install
colcon test-result --test-result-base build --verbose
```

The build script deliberately uses `/usr/bin/python3`, matching the ROS 2
Jazzy Python ABI even when Conda is active.

## Depth-angular closed loop

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch pc_gvf depth_angular_demo.launch.py
```

This starts RViz2 by default with the robot marker, velocity arrow, executed
path, odometry, and forward depth image. Disable the GUI for headless runs:

```bash
ros2 launch pc_gvf depth_angular_demo.launch.py rviz:=false
```

The default scenario is `single_pillar`. Available migrated scenarios are
`empty`, `single_pillar`, `offset_box`, `center_sphere`, `overhead_bar`,
`diagonal_gap`, and `narrow_gate`:

```bash
ros2 launch pc_gvf depth_angular_demo.launch.py scenario:=single_pillar
ros2 launch pc_gvf depth_angular_demo.launch.py scenario:=narrow_gate
```

RViz shows the scenario geometry, camera FOV, unsafe angular cells, depth
hits, reference/goal/command rays, and the angular GVF vectors. The vectors
retain the ROS 1 depth-angular visualization: arrow length and red/green
color encode safe speed, with animated highlights along each arrow.
The robot uses GVF-Nav's hummingbird quadrotor mesh, installed in
`pc_gvf_platforms`; its pose follows odometry and `robot_scale` defaults to 1.0.
Ground-platform launches continue to use their box marker.

The port uses ROS 2 message construction, ROS clock timestamps, and
transient-local MarkerArray publishers (the ROS 2 equivalent of latched
markers). FOV points are transformed into the world frame using the current
body attitude and camera optical rotation; no ROS 1 TF node is required.

The demo publishes synthetic odometry, depth, CameraInfo, and human intent.
The default controller is the C++ runtime and publishes
`pc_gvf_msgs/msg/PositionCommand`. Inspect it with:

```bash
ros2 topic echo --once /depth_angular_controller/status
ros2 topic echo /position_cmd --field velocity
```

Connect the controller to another simulator or robot with:

```bash
ros2 run pc_gvf depth_angular_controller --ros-args \
  -p odom_topic:=/your/odom \
  -p depth_topic:=/your/depth/image_raw \
  -p camera_info_topic:=/your/depth/camera_info \
  -p human_intent_topic:=/human_intent \
  -p cmd_topic:=/position_cmd
```

Depth input supports `32FC1` metres and `16UC1` millimetres.

## Ground-platform adapters

Launch the PositionCommand-to-Twist bridge and a differential-drive simulator:

```bash
ros2 launch pc_gvf_platforms ground_platform.launch.py
```

This launch also starts RViz2 by default and displays TF, the ground-robot
box, commanded velocity arrow, odometry, and executed path. Use `rviz:=false`
for headless execution.

Use an installed ROS 2 joystick driver together with:

```bash
ros2 run pc_gvf_platforms joy_to_intent
```

Replay a deterministic intent trace directly as `TwistStamped`:

```bash
ros2 run pc_gvf_platforms intent_trace_replay --ros-args \
  -p trace_file:=$(ros2 pkg prefix pc_gvf_platforms)/share/pc_gvf_platforms/config/forward_stop.yaml
```

See [MIGRATION.md](MIGRATION.md) for the mapping from the old ROS 1 packages.
