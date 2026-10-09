
### Run p5_retain_goal_only_confirm_20261008_baseline_ego1p5_1

- Started: 2026-10-08 11:12:36 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `goal`

#### Isaac command application — p5_retain_goal_only_confirm_20261008_baseline_ego1p5_1

- Result: ARRIVED
- Wall duration / simulation duration: 75.501 / 72.983 s
- Applied simulation control frames: 4381
- Runtime FPS (simulation time / wall time): 60.027 / 58.026 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.160 / 15.977 / 18.048 / 79.433 ms
- Distinct applied command values: 3836
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 19.300 / 16.198 / 32.883 / 1943.326 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_retain_goal_only_confirm_20261008_baseline_ego1p5_1

- Wall duration: 89.447 s
- Timer callbacks / command frames / guidance frames: 4369 / 4369 / 4179
- Command FPS (ROS simulation time / active wall time): 59.849 / 57.861 Hz
- Guidance FPS (active wall time): 57.967 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.709 / 16.667 / 16.667 / 133.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.844 / 1.421 / 4.602 / 177.382 ms
- Command interval mean / P50 / P95 / max: 17.283 / 15.961 / 22.756 / 377.398 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 42.562 / 33.333 / 66.667 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.715 / 3.010 / 8.572 / 183.973 ms (steady clock)
- DDS publish call mean / P95 / max: 0.071 / 0.426 / 1.677 ms (steady clock)

#### Command bridge — p5_retain_goal_only_confirm_20261008_baseline_ego1p5_1

- Wall duration: 89.337 s
- Received / published control frames: 4369 / 4368
- Published control FPS (ROS simulation time / active wall time): 59.849 / 57.848 Hz
- Command stamp interval mean / P50 / P95 / max: 16.709 / 16.667 / 16.667 / 133.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.130 / 0.000 / 0.000 / 133.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.283 / 15.978 / 22.752 / 377.279 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.089 / 0.085 / 0.115 / 1.251 ms (steady clock)
