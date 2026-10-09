
### Run p5_goal_polish_explore_20261007_before_ego1p5_3

- Started: 2026-10-07 23:08:30 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `goal`

#### Isaac command application — p5_goal_polish_explore_20261007_before_ego1p5_3

- Result: ARRIVED
- Wall duration / simulation duration: 73.570 / 71.317 s
- Applied simulation control frames: 4281
- Runtime FPS (simulation time / wall time): 60.028 / 58.189 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.108 / 15.761 / 17.942 / 80.840 ms
- Distinct applied command values: 3118
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 23.144 / 15.955 / 32.808 / 4294.426 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — p5_goal_polish_explore_20261007_before_ego1p5_3

- Wall duration: 87.401 s
- Received / published control frames: 4263 / 4263
- Published control FPS (ROS simulation time / active wall time): 59.776 / 58.233 Hz
- Command stamp interval mean / P50 / P95 / max: 16.729 / 16.667 / 16.667 / 50.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.164 / 0.000 / 0.000 / 50.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.172 / 15.831 / 23.061 / 88.623 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.087 / 0.084 / 0.110 / 0.942 ms (steady clock)

#### Controller — p5_goal_polish_explore_20261007_before_ego1p5_3

- Wall duration: 87.508 s
- Timer callbacks / command frames / guidance frames: 4264 / 4263 / 4073
- Command FPS (ROS simulation time / active wall time): 59.776 / 58.233 Hz
- Guidance FPS (active wall time): 58.103 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.729 / 16.667 / 16.667 / 50.000 ms
- Avoidance compute mean / P50 / P95 / max: 1.357 / 0.814 / 3.485 / 48.773 ms
- Command interval mean / P50 / P95 / max: 17.172 / 15.845 / 23.095 / 88.481 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 43.449 / 33.333 / 83.333 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.387 / 2.747 / 8.470 / 49.986 ms (steady clock)
- DDS publish call mean / P95 / max: 0.062 / 0.386 / 1.722 ms (steady clock)
