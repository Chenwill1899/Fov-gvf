
### Run final_v5_recovery_ego1p3_1

- Started: 2026-10-06 22:44:46 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v5_recovery_ego1p3_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 73.216 / 72.017 s
- Applied simulation control frames: 4323
- Runtime FPS (simulation time / wall time): 60.028 / 59.045 Hz
- Isaac frame interval mean / P50 / P95 / max: 16.813 / 15.390 / 17.484 / 79.347 ms
- Distinct applied command values: 2270
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 30.430 / 15.805 / 32.359 / 4459.473 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — final_v5_recovery_ego1p3_1

- Wall duration: 86.735 s
- Timer callbacks / command frames / guidance frames: 4319 / 4318 / 3237
- Command FPS (ROS simulation time / active wall time): 59.958 / 59.440 Hz
- Guidance FPS (active wall time): 47.462 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.678 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 2.423 / 1.584 / 6.915 / 15.465 ms
- Command interval mean / P50 / P95 / max: 16.824 / 15.220 / 23.321 / 86.485 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 38.075 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 4.070 / 3.717 / 10.582 / 31.789 ms (steady clock)
- DDS publish call mean / P95 / max: 0.056 / 0.245 / 1.700 ms (steady clock)

#### Command bridge — final_v5_recovery_ego1p3_1

- Wall duration: 86.631 s
- Received / published control frames: 4318 / 4318
- Published control FPS (ROS simulation time / active wall time): 59.958 / 59.440 Hz
- Command stamp interval mean / P50 / P95 / max: 16.678 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.289 / 0.000 / 0.000 / 33.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 16.824 / 15.200 / 23.204 / 86.661 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.082 / 0.080 / 0.103 / 0.455 ms (steady clock)
