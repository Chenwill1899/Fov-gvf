
### Run p5_retain_goal_only_confirm_20261008_v4_ego1p5_3

- Started: 2026-10-08 11:21:32 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `goal`

#### Isaac command application — p5_retain_goal_only_confirm_20261008_v4_ego1p5_3

- Result: ARRIVED
- Wall duration / simulation duration: 73.059 / 71.183 s
- Applied simulation control frames: 4273
- Runtime FPS (simulation time / wall time): 60.028 / 58.487 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.022 / 15.819 / 17.773 / 78.965 ms
- Distinct applied command values: 3189
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.470 / 16.024 / 32.706 / 4537.270 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_retain_goal_only_confirm_20261008_v4_ego1p5_3

- Wall duration: 87.346 s
- Timer callbacks / command frames / guidance frames: 4268 / 4267 / 4078
- Command FPS (ROS simulation time / active wall time): 59.944 / 58.693 Hz
- Guidance FPS (active wall time): 58.597 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.682 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.438 / 0.925 / 4.311 / 30.674 ms
- Command interval mean / P50 / P95 / max: 17.038 / 15.922 / 22.050 / 97.688 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 39.190 / 33.333 / 66.667 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.475 / 3.151 / 8.240 / 31.276 ms (steady clock)
- DDS publish call mean / P95 / max: 0.070 / 0.435 / 1.510 ms (steady clock)

#### Command bridge — p5_retain_goal_only_confirm_20261008_v4_ego1p5_3

- Wall duration: 87.241 s
- Received / published control frames: 4267 / 4267
- Published control FPS (ROS simulation time / active wall time): 59.944 / 58.694 Hz
- Command stamp interval mean / P50 / P95 / max: 16.682 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.086 / 0.000 / 0.000 / 33.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.038 / 15.916 / 22.088 / 97.760 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.088 / 0.084 / 0.112 / 1.468 ms (steady clock)
