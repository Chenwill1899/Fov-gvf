# ROS 1 to ROS 2 migration map

This workspace targets ROS 2 Jazzy and replaces the old catkin workspace with
four ament packages:

| ROS 2 package | Responsibility |
|---|---|
| `pc_gvf_msgs` | ROSIDL `PositionCommand` message |
| `pc_gvf_core` | Self-contained 2D/3D Cartesian fluid solvers, depth-grid observation, and C++ regression tests |
| `pc_gvf` | ROS-independent depth-angular algorithm, rclpy controller, depth demo, and launch |
| `pc_gvf_platforms` | PositionCommand/Twist bridge, holonomic and differential-drive simulation, Joy mapping, and trace replay |

## Legacy package disposition

| ROS 1 package/group | ROS 2 disposition |
|---|---|
| `fluid` numerical solvers and depth observation | Ported into `pc_gvf_core`; the coarse/fine boundary fix is included |
| depth-angular Python controller | Ported from `rospy` to `rclpy` in `pc_gvf` |
| `quadrotor_msgs/PositionCommand` | Ported to `pc_gvf_msgs` |
| `gvf_cmd_bridge` | Ported to `pc_gvf_platforms/command_bridge.py` |
| `b2_gvf_sim`, `diff_drive_gvf_sim` | Replaced by the mode-selectable ROS 2 `kinematic_sim` |
| `human_input_sim` | Ported as `joy_to_intent` and direct `intent_trace_replay` |
| vendored `joy`, `joystick_drivers`, `ps3joy` | Replaced by the ROS 2 `joy`/platform driver packages |
| custom RViz plugins and odometry visualizers | Replaced by `navigation_visualizer`, standard TF/Marker/Path/Image displays, and installed RViz2 configurations |
| `dynamic_map_generator`, `mockamap` | The reproducible depth demo moved into `depth_angular_demo_sim`; real deployments consume camera topics directly |
| legacy `common_msgs`, `controller_msgs`, unused quadrotor messages | Removed from the maintained interface because the current guidance chain does not consume them |
| `plan_env` | Its ROS-independent fluid algorithms are retained in `pc_gvf_core`; the old ROS 1 ESDF node is not part of the depth/FOV runtime |
| ROS 1 SO3 simulator/control and `drone_control` | Replaced at the deployment boundary; connect `/position_cmd` through a ROS 2 PX4 or vehicle-specific adapter |
| miscellaneous ROS 1 utility packages | Replaced by ROS 2 standard messages, TF2, launch, parameters, and platform packages |

The final deployment adapter is intentionally vehicle-specific: PX4, B2, and
generic differential-drive systems have different ROS 2 APIs and safety
contracts. The repository provides the stable command and sensor boundary
instead of embedding one obsolete ROS 1 flight stack.

## Verified behavior

- Clean `colcon build` from this directory, without the old repository.
- C++ depth projection, Cartesian guidance, and coarse/fine boundary tests.
- Python angular-field, depth decoder, joystick, trace, and platform math tests.
- Live ROS 2 depth demo reaches `NAVIGATING` and publishes PositionCommand.
- Both launch files start RViz2 by default; RViz subscribes to the configured
  Marker/Path and depth or TF topics.
- The depth launch exposes all migrated analytic test scenes, defaults to
  `single_pillar`, and publishes scenario, FOV, depth-hit, unsafe-direction,
  reference-ray, command-ray, and angular-GVF markers.
- Live ground bridge clamps a 1.0 m/s PositionCommand to 0.6 m/s Twist; the
  simulator publishes the resulting odometry and exits cleanly.
