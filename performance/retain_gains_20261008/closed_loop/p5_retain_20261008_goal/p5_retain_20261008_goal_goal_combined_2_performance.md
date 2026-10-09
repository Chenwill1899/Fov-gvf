
### Run p5_retain_20261008_goal_combined_ego1p5_2

- Started: 2026-10-08 10:35:26 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `goal`

#### Isaac command application — p5_retain_20261008_goal_combined_ego1p5_2

- Result: ARRIVED
- Wall duration / simulation duration: 75.326 / 72.883 s
- Applied simulation control frames: 4375
- Runtime FPS (simulation time / wall time): 60.027 / 58.081 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.144 / 15.693 / 17.735 / 79.060 ms
- Distinct applied command values: 3904
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 18.933 / 15.822 / 32.467 / 1793.335 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — p5_retain_20261008_goal_combined_ego1p5_2

- Wall duration: 89.125 s
- Received / published control frames: 4361 / 4361
- Published control FPS (ROS simulation time / active wall time): 59.835 / 58.173 Hz
- Command stamp interval mean / P50 / P95 / max: 16.713 / 16.667 / 16.667 / 83.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.099 / 0.000 / 0.000 / 83.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.190 / 15.668 / 23.250 / 94.613 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.088 / 0.084 / 0.112 / 1.548 ms (steady clock)

#### Controller — p5_retain_20261008_goal_combined_ego1p5_2

- Wall duration: 89.233 s
- Timer callbacks / command frames / guidance frames: 4362 / 4361 / 4171
- Command FPS (ROS simulation time / active wall time): 59.835 / 58.173 Hz
- Guidance FPS (active wall time): 57.828 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.713 / 16.667 / 16.667 / 83.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.503 / 1.102 / 3.625 / 80.280 ms
- Command interval mean / P50 / P95 / max: 17.190 / 15.663 / 23.392 / 94.210 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 47.087 / 50.000 / 83.333 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.395 / 2.701 / 8.591 / 80.451 ms (steady clock)
- DDS publish call mean / P95 / max: 0.072 / 0.440 / 1.266 ms (steady clock)
