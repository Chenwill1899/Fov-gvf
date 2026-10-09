
### Run p5_retain_goal_only_confirm_20261008_v4_ego1p5_2

- Started: 2026-10-08 11:16:58 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `goal`

#### Isaac command application — p5_retain_goal_only_confirm_20261008_v4_ego1p5_2

- Result: ARRIVED
- Wall duration / simulation duration: 76.807 / 74.033 s
- Applied simulation control frames: 4444
- Runtime FPS (simulation time / wall time): 60.027 / 57.859 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.210 / 15.868 / 17.912 / 80.616 ms
- Distinct applied command values: 4006
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 18.821 / 16.028 / 32.494 / 1739.621 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — p5_retain_goal_only_confirm_20261008_v4_ego1p5_2

- Wall duration: 90.644 s
- Received / published control frames: 4395 / 4395
- Published control FPS (ROS simulation time / active wall time): 59.365 / 57.491 Hz
- Command stamp interval mean / P50 / P95 / max: 16.845 / 16.667 / 16.667 / 83.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.372 / 0.000 / 0.000 / 83.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.394 / 16.133 / 22.143 / 86.196 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.087 / 0.083 / 0.111 / 1.284 ms (steady clock)

#### Controller — p5_retain_goal_only_confirm_20261008_v4_ego1p5_2

- Wall duration: 90.753 s
- Timer callbacks / command frames / guidance frames: 4396 / 4395 / 4205
- Command FPS (ROS simulation time / active wall time): 59.365 / 57.491 Hz
- Guidance FPS (active wall time): 57.088 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.845 / 16.667 / 16.667 / 83.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.831 / 1.125 / 4.475 / 76.659 ms
- Command interval mean / P50 / P95 / max: 17.394 / 16.138 / 22.234 / 86.232 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 36.829 / 33.333 / 50.000 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.774 / 3.262 / 8.241 / 86.142 ms (steady clock)
- DDS publish call mean / P95 / max: 0.068 / 0.426 / 1.768 ms (steady clock)
