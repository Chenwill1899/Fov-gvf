
### Run p5_enable_goal_20261008_baseline_ego1p5_1

- Started: 2026-10-08 12:41:53 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `goal`

#### Isaac command application — p5_enable_goal_20261008_baseline_ego1p5_1

- Result: ARRIVED
- Wall duration / simulation duration: 74.937 / 71.200 s
- Applied simulation control frames: 4274
- Runtime FPS (simulation time / wall time): 60.028 / 57.034 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.458 / 15.935 / 18.130 / 79.753 ms
- Distinct applied command values: 3342
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.944 / 16.136 / 32.839 / 3638.070 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — p5_enable_goal_20261008_baseline_ego1p5_1

- Wall duration: 88.922 s
- Received / published control frames: 4261 / 4261
- Published control FPS (ROS simulation time / active wall time): 59.845 / 57.135 Hz
- Command stamp interval mean / P50 / P95 / max: 16.710 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.145 / 0.000 / 0.000 / 33.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.502 / 16.203 / 23.335 / 87.748 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.092 / 0.087 / 0.115 / 1.870 ms (steady clock)

#### Controller — p5_enable_goal_20261008_baseline_ego1p5_1

- Wall duration: 89.032 s
- Timer callbacks / command frames / guidance frames: 4262 / 4261 / 4071
- Command FPS (ROS simulation time / active wall time): 59.845 / 57.135 Hz
- Guidance FPS (active wall time): 57.131 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.710 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.321 / 0.906 / 3.443 / 40.299 ms
- Command interval mean / P50 / P95 / max: 17.502 / 16.207 / 23.331 / 87.591 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 39.151 / 33.333 / 66.667 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.364 / 2.822 / 8.229 / 40.423 ms (steady clock)
- DDS publish call mean / P95 / max: 0.068 / 0.423 / 2.036 ms (steady clock)
