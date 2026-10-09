
### Run final_v17_recovery_ego1p3_1

- Started: 2026-10-07 00:30:33 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v17_recovery_ego1p3_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 74.593 / 72.017 s
- Applied simulation control frames: 4323
- Runtime FPS (simulation time / wall time): 60.028 / 57.954 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.133 / 15.751 / 17.988 / 79.216 ms
- Distinct applied command values: 2138
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 32.748 / 16.349 / 32.940 / 5874.937 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — final_v17_recovery_ego1p3_1

- Wall duration: 88.356 s
- Timer callbacks / command frames / guidance frames: 4320 / 4319 / 3238
- Command FPS (ROS simulation time / active wall time): 59.972 / 58.339 Hz
- Guidance FPS (active wall time): 46.920 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.674 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.079 / 0.390 / 4.994 / 13.943 ms
- Command interval mean / P50 / P95 / max: 17.141 / 16.072 / 23.251 / 85.904 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 37.523 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.132 / 2.880 / 8.317 / 22.366 ms (steady clock)
- DDS publish call mean / P95 / max: 0.061 / 0.385 / 1.488 ms (steady clock)

#### Command bridge — final_v17_recovery_ego1p3_1

- Wall duration: 88.250 s
- Received / published control frames: 4319 / 4319
- Published control FPS (ROS simulation time / active wall time): 59.972 / 58.339 Hz
- Command stamp interval mean / P50 / P95 / max: 16.674 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.054 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.141 / 16.078 / 23.273 / 85.971 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.085 / 0.083 / 0.110 / 0.653 ms (steady clock)
