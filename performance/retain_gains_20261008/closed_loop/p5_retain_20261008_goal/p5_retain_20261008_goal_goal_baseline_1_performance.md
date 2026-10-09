
### Run p5_retain_20261008_goal_baseline_ego1p5_1

- Started: 2026-10-08 10:09:43 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `goal`

#### Isaac command application — p5_retain_20261008_goal_baseline_ego1p5_1

- Result: ARRIVED
- Wall duration / simulation duration: 73.170 / 71.217 s
- Applied simulation control frames: 4275
- Runtime FPS (simulation time / wall time): 60.028 / 58.425 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.041 / 15.729 / 17.757 / 78.374 ms
- Distinct applied command values: 3329
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.525 / 15.954 / 32.812 / 2556.675 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_retain_20261008_goal_baseline_ego1p5_1

- Wall duration: 87.032 s
- Timer callbacks / command frames / guidance frames: 4263 / 4262 / 4072
- Command FPS (ROS simulation time / active wall time): 59.846 / 58.533 Hz
- Guidance FPS (active wall time): 58.250 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.710 / 16.667 / 16.667 / 50.000 ms
- Avoidance compute mean / P50 / P95 / max: 1.442 / 0.890 / 3.810 / 40.344 ms
- Command interval mean / P50 / P95 / max: 17.084 / 15.633 / 23.056 / 88.362 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 38.335 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.432 / 2.975 / 8.402 / 41.247 ms (steady clock)
- DDS publish call mean / P95 / max: 0.069 / 0.425 / 1.727 ms (steady clock)

#### Command bridge — p5_retain_20261008_goal_baseline_ego1p5_1

- Wall duration: 86.922 s
- Received / published control frames: 4262 / 4262
- Published control FPS (ROS simulation time / active wall time): 59.846 / 58.533 Hz
- Command stamp interval mean / P50 / P95 / max: 16.710 / 16.667 / 16.667 / 50.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.215 / 0.000 / 0.000 / 50.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.084 / 15.670 / 22.944 / 88.457 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.089 / 0.086 / 0.113 / 1.314 ms (steady clock)
