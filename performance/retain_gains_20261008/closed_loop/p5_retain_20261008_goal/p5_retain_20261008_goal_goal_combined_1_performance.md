
### Run p5_retain_20261008_goal_combined_ego1p5_1

- Started: 2026-10-08 10:17:02 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `goal`

#### Isaac command application — p5_retain_20261008_goal_combined_ego1p5_1

- Result: ARRIVED
- Wall duration / simulation duration: 74.069 / 71.183 s
- Applied simulation control frames: 4273
- Runtime FPS (simulation time / wall time): 60.028 / 57.689 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.244 / 15.809 / 17.836 / 90.357 ms
- Distinct applied command values: 3236
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.378 / 16.004 / 32.625 / 4659.162 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_retain_20261008_goal_combined_ego1p5_1

- Wall duration: 88.021 s
- Timer callbacks / command frames / guidance frames: 4269 / 4268 / 4077
- Command FPS (ROS simulation time / active wall time): 59.958 / 57.949 Hz
- Guidance FPS (active wall time): 57.805 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.678 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.298 / 0.894 / 3.510 / 20.342 ms
- Command interval mean / P50 / P95 / max: 17.256 / 15.902 / 22.305 / 99.024 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 41.640 / 33.333 / 66.667 / 83.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.357 / 3.005 / 7.657 / 24.640 ms (steady clock)
- DDS publish call mean / P95 / max: 0.068 / 0.426 / 1.597 ms (steady clock)

#### Command bridge — p5_retain_20261008_goal_combined_ego1p5_1

- Wall duration: 87.913 s
- Received / published control frames: 4268 / 4268
- Published control FPS (ROS simulation time / active wall time): 59.958 / 57.950 Hz
- Command stamp interval mean / P50 / P95 / max: 16.678 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.086 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.256 / 15.891 / 22.275 / 99.013 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.088 / 0.085 / 0.112 / 1.133 ms (steady clock)
