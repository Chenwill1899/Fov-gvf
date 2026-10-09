
### Run p5_enable_goal_20261008_memory_ego1p5_1

- Started: 2026-10-08 12:44:55 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `goal`

#### Isaac command application — p5_enable_goal_20261008_memory_ego1p5_1

- Result: ARRIVED
- Wall duration / simulation duration: 77.386 / 73.683 s
- Applied simulation control frames: 4423
- Runtime FPS (simulation time / wall time): 60.027 / 57.155 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.421 / 16.044 / 18.360 / 87.570 ms
- Distinct applied command values: 3319
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.844 / 16.260 / 34.632 / 3684.298 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_enable_goal_20261008_memory_ego1p5_1

- Wall duration: 91.343 s
- Timer callbacks / command frames / guidance frames: 4337 / 4336 / 4146
- Command FPS (ROS simulation time / active wall time): 58.846 / 56.299 Hz
- Guidance FPS (active wall time): 56.009 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.993 / 16.667 / 16.667 / 83.333 ms
- Avoidance compute mean / P50 / P95 / max: 2.132 / 0.965 / 7.617 / 64.449 ms
- Command interval mean / P50 / P95 / max: 17.762 / 16.210 / 26.920 / 94.526 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 43.990 / 33.333 / 83.333 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 4.786 / 3.934 / 13.689 / 73.102 ms (steady clock)
- DDS publish call mean / P95 / max: 0.059 / 0.223 / 1.959 ms (steady clock)

#### Command bridge — p5_enable_goal_20261008_memory_ego1p5_1

- Wall duration: 91.234 s
- Received / published control frames: 4336 / 4336
- Published control FPS (ROS simulation time / active wall time): 58.846 / 56.299 Hz
- Command stamp interval mean / P50 / P95 / max: 16.993 / 16.667 / 16.667 / 83.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.930 / 0.000 / 0.000 / 83.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.762 / 16.232 / 26.863 / 94.500 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.088 / 0.084 / 0.115 / 0.689 ms (steady clock)
