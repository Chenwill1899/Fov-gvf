
### Run explore_v1_sweep_ego1p3_1

- Started: 2026-10-06 22:07:21 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — explore_v1_sweep_ego1p3_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 41.548 / 40.017 s
- Applied simulation control frames: 2403
- Runtime FPS (simulation time / wall time): 60.050 / 57.837 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.155 / 15.814 / 18.006 / 78.403 ms
- Distinct applied command values: 2095
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 18.749 / 16.011 / 31.299 / 1060.205 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — explore_v1_sweep_ego1p3_1

- Wall duration: 55.402 s
- Timer callbacks / command frames / guidance frames: 2385 / 2385 / 2204
- Command FPS (ROS simulation time / active wall time): 59.575 / 57.394 Hz
- Guidance FPS (active wall time): 57.673 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.786 / 16.667 / 16.667 / 66.667 ms
- Avoidance compute mean / P50 / P95 / max: 1.960 / 1.548 / 4.221 / 55.386 ms
- Command interval mean / P50 / P95 / max: 17.424 / 15.669 / 24.754 / 361.553 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 43.103 / 33.333 / 83.333 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.932 / 2.978 / 9.932 / 55.948 ms (steady clock)
- DDS publish call mean / P95 / max: 0.075 / 0.441 / 2.646 ms (steady clock)

#### Command bridge — explore_v1_sweep_ego1p3_1

- Wall duration: 54.963 s
- Received / published control frames: 2385 / 2385
- Published control FPS (ROS simulation time / active wall time): 59.575 / 57.394 Hz
- Command stamp interval mean / P50 / P95 / max: 16.786 / 16.667 / 16.667 / 66.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.294 / 0.000 / 0.000 / 66.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.424 / 15.673 / 24.759 / 361.877 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.087 / 0.083 / 0.114 / 1.005 ms (steady clock)
