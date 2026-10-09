
### Run final_v17_recovery_ego1p2_1

- Started: 2026-10-07 00:29:02 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v17_recovery_ego1p2_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 75.746 / 72.017 s
- Applied simulation control frames: 4323
- Runtime FPS (simulation time / wall time): 60.028 / 57.072 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.396 / 15.969 / 18.433 / 82.778 ms
- Distinct applied command values: 1956
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 36.477 / 16.659 / 35.997 / 5691.011 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — final_v17_recovery_ego1p2_1

- Wall duration: 89.607 s
- Timer callbacks / command frames / guidance frames: 4234 / 4233 / 3152
- Command FPS (ROS simulation time / active wall time): 58.778 / 56.313 Hz
- Guidance FPS (active wall time): 44.854 Hz
- Command interval in ROS time mean / P50 / P95 / max: 17.013 / 16.667 / 16.667 / 450.000 ms
- Avoidance compute mean / P50 / P95 / max: 1.208 / 0.475 / 1.281 / 482.283 ms
- Command interval mean / P50 / P95 / max: 17.758 / 16.066 / 24.569 / 494.804 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 65.186 / 66.667 / 116.667 / 133.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.387 / 2.408 / 8.555 / 483.470 ms (steady clock)
- DDS publish call mean / P95 / max: 0.062 / 0.321 / 1.700 ms (steady clock)

#### Command bridge — final_v17_recovery_ego1p2_1

- Wall duration: 89.162 s
- Received / published control frames: 4233 / 4228
- Published control FPS (ROS simulation time / active wall time): 58.778 / 56.246 Hz
- Command stamp interval mean / P50 / P95 / max: 17.013 / 16.667 / 16.667 / 450.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.614 / 0.000 / 0.000 / 450.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.758 / 16.064 / 24.546 / 494.370 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.092 / 0.088 / 0.117 / 1.542 ms (steady clock)
