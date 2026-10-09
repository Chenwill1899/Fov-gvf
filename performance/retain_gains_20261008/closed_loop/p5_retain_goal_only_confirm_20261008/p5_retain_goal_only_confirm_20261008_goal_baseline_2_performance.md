
### Run p5_retain_goal_only_confirm_20261008_baseline_ego1p5_2

- Started: 2026-10-08 11:18:30 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `goal`

#### Isaac command application — p5_retain_goal_only_confirm_20261008_baseline_ego1p5_2

- Result: ARRIVED
- Wall duration / simulation duration: 75.883 / 73.083 s
- Applied simulation control frames: 4387
- Runtime FPS (simulation time / wall time): 60.027 / 57.813 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.224 / 15.982 / 17.939 / 80.446 ms
- Distinct applied command values: 3914
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 19.004 / 16.160 / 32.732 / 1806.597 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — p5_retain_goal_only_confirm_20261008_baseline_ego1p5_2

- Wall duration: 89.717 s
- Received / published control frames: 4360 / 4360
- Published control FPS (ROS simulation time / active wall time): 59.658 / 57.729 Hz
- Command stamp interval mean / P50 / P95 / max: 16.762 / 16.667 / 16.667 / 100.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.172 / 0.000 / 0.000 / 100.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.322 / 16.067 / 22.422 / 90.183 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.089 / 0.084 / 0.112 / 1.562 ms (steady clock)

#### Controller — p5_retain_goal_only_confirm_20261008_baseline_ego1p5_2

- Wall duration: 89.827 s
- Timer callbacks / command frames / guidance frames: 4361 / 4360 / 4169
- Command FPS (ROS simulation time / active wall time): 59.658 / 57.729 Hz
- Guidance FPS (active wall time): 57.490 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.762 / 16.667 / 16.667 / 100.000 ms
- Avoidance compute mean / P50 / P95 / max: 1.663 / 1.103 / 4.156 / 86.455 ms
- Command interval mean / P50 / P95 / max: 17.322 / 16.040 / 22.464 / 90.212 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 41.877 / 33.333 / 83.333 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.573 / 3.056 / 8.443 / 90.135 ms (steady clock)
- DDS publish call mean / P95 / max: 0.072 / 0.432 / 1.337 ms (steady clock)
