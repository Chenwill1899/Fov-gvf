
### Run p5_retain_20261008_goal_baseline_ego1p5_3

- Started: 2026-10-08 10:50:09 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `goal`

#### Isaac command application — p5_retain_20261008_goal_baseline_ego1p5_3

- Result: ARRIVED
- Wall duration / simulation duration: 73.081 / 71.250 s
- Applied simulation control frames: 4277
- Runtime FPS (simulation time / wall time): 60.028 / 58.524 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.011 / 15.846 / 17.763 / 79.865 ms
- Distinct applied command values: 3310
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.615 / 16.053 / 32.805 / 2864.653 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — p5_retain_20261008_goal_baseline_ego1p5_3

- Wall duration: 86.936 s
- Received / published control frames: 4269 / 4269
- Published control FPS (ROS simulation time / active wall time): 59.902 / 58.411 Hz
- Command stamp interval mean / P50 / P95 / max: 16.694 / 16.667 / 16.667 / 50.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.152 / 0.000 / 0.000 / 50.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.120 / 16.188 / 21.670 / 364.787 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.088 / 0.084 / 0.112 / 1.599 ms (steady clock)

#### Controller — p5_retain_20261008_goal_baseline_ego1p5_3

- Wall duration: 87.045 s
- Timer callbacks / command frames / guidance frames: 4269 / 4269 / 4078
- Command FPS (ROS simulation time / active wall time): 59.902 / 58.411 Hz
- Guidance FPS (active wall time): 58.435 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.694 / 16.667 / 16.667 / 50.000 ms
- Avoidance compute mean / P50 / P95 / max: 1.384 / 0.883 / 3.631 / 47.738 ms
- Command interval mean / P50 / P95 / max: 17.120 / 16.224 / 21.671 / 364.400 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 39.378 / 33.333 / 66.667 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.445 / 3.071 / 8.053 / 50.086 ms (steady clock)
- DDS publish call mean / P95 / max: 0.070 / 0.433 / 1.748 ms (steady clock)
