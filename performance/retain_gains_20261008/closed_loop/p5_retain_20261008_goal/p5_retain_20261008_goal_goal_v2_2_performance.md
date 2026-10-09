
### Run p5_retain_20261008_goal_v2_ego1p5_2

- Started: 2026-10-08 10:24:25 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `goal`

#### Isaac command application — p5_retain_20261008_goal_v2_ego1p5_2

- Result: ARRIVED
- Wall duration / simulation duration: 74.338 / 71.267 s
- Applied simulation control frames: 4278
- Runtime FPS (simulation time / wall time): 60.028 / 57.548 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.301 / 15.813 / 17.830 / 78.375 ms
- Distinct applied command values: 3106
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 23.422 / 16.053 / 33.160 / 4336.779 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — p5_retain_20261008_goal_v2_ego1p5_2

- Wall duration: 88.528 s
- Received / published control frames: 4270 / 4270
- Published control FPS (ROS simulation time / active wall time): 59.902 / 57.437 Hz
- Command stamp interval mean / P50 / P95 / max: 16.694 / 16.667 / 16.667 / 50.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.109 / 0.000 / 0.000 / 50.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.410 / 15.754 / 24.157 / 363.642 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.088 / 0.084 / 0.112 / 1.680 ms (steady clock)

#### Controller — p5_retain_20261008_goal_v2_ego1p5_2

- Wall duration: 88.635 s
- Timer callbacks / command frames / guidance frames: 4270 / 4270 / 4079
- Command FPS (ROS simulation time / active wall time): 59.902 / 57.438 Hz
- Guidance FPS (active wall time): 57.574 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.694 / 16.667 / 16.667 / 50.000 ms
- Avoidance compute mean / P50 / P95 / max: 1.287 / 0.793 / 3.387 / 36.067 ms
- Command interval mean / P50 / P95 / max: 17.410 / 15.758 / 24.148 / 363.273 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 40.774 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.265 / 2.232 / 8.800 / 39.836 ms (steady clock)
- DDS publish call mean / P95 / max: 0.068 / 0.430 / 2.183 ms (steady clock)
