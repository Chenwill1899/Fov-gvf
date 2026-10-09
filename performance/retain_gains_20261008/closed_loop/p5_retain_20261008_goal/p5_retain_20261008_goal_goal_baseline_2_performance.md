
### Run p5_retain_20261008_goal_baseline_ego1p5_2

- Started: 2026-10-08 10:31:45 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `goal`

#### Isaac command application — p5_retain_20261008_goal_baseline_ego1p5_2

- Result: ARRIVED
- Wall duration / simulation duration: 74.625 / 71.217 s
- Applied simulation control frames: 4275
- Runtime FPS (simulation time / wall time): 60.028 / 57.287 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.380 / 15.864 / 17.906 / 80.500 ms
- Distinct applied command values: 3153
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 23.163 / 16.054 / 32.719 / 5113.519 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — p5_retain_20261008_goal_baseline_ego1p5_2

- Wall duration: 88.427 s
- Received / published control frames: 4271 / 4271
- Published control FPS (ROS simulation time / active wall time): 59.972 / 57.514 Hz
- Command stamp interval mean / P50 / P95 / max: 16.674 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.047 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.387 / 16.112 / 22.330 / 83.366 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.091 / 0.086 / 0.116 / 1.483 ms (steady clock)

#### Controller — p5_retain_20261008_goal_baseline_ego1p5_2

- Wall duration: 88.536 s
- Timer callbacks / command frames / guidance frames: 4272 / 4271 / 4081
- Command FPS (ROS simulation time / active wall time): 59.972 / 57.514 Hz
- Guidance FPS (active wall time): 57.342 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.674 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.264 / 0.813 / 3.585 / 22.934 ms
- Command interval mean / P50 / P95 / max: 17.387 / 16.120 / 22.350 / 83.341 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 38.777 / 33.333 / 66.667 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.319 / 3.221 / 7.843 / 30.088 ms (steady clock)
- DDS publish call mean / P95 / max: 0.067 / 0.420 / 1.580 ms (steady clock)
