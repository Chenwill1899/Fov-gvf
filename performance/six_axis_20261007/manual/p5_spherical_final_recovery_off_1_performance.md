
### Run p5_spherical_final_recovery_off_1

- Started: 2026-10-07 17:01:40 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_spherical_final_recovery_off_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 73.869 / 72.017 s
- Applied simulation control frames: 4323
- Runtime FPS (simulation time / wall time): 60.028 / 58.523 Hz
- Isaac frame interval mean / P50 / P95 / max: 16.948 / 15.673 / 17.730 / 79.187 ms
- Distinct applied command values: 2124
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 32.707 / 16.265 / 32.547 / 5622.552 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_spherical_final_recovery_off_1

- Wall duration: 87.653 s
- Timer callbacks / command frames / guidance frames: 4319 / 4319 / 3238
- Command FPS (ROS simulation time / active wall time): 59.958 / 58.463 Hz
- Guidance FPS (active wall time): 47.220 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.678 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.034 / 0.384 / 4.721 / 13.601 ms
- Command interval mean / P50 / P95 / max: 17.105 / 15.763 / 22.895 / 642.088 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 39.397 / 33.333 / 83.333 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.190 / 2.987 / 8.668 / 19.381 ms (steady clock)
- DDS publish call mean / P95 / max: 0.056 / 0.179 / 1.560 ms (steady clock)

#### Command bridge — p5_spherical_final_recovery_off_1

- Wall duration: 87.545 s
- Received / published control frames: 4319 / 4319
- Published control FPS (ROS simulation time / active wall time): 59.958 / 58.463 Hz
- Command stamp interval mean / P50 / P95 / max: 16.678 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.031 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.105 / 15.726 / 22.849 / 642.297 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.084 / 0.081 / 0.106 / 1.234 ms (steady clock)
