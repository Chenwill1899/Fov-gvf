
### Run p5_retain_20261008_jitter_jitter_combined_2

- Started: 2026-10-08 10:32:36 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_retain_20261008_jitter_jitter_combined_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 37.671 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 54.233 Hz
- Isaac frame interval mean / P50 / P95 / max: 18.282 / 16.752 / 19.038 / 79.008 ms
- Distinct applied command values: 1538
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 23.000 / 16.937 / 34.617 / 2102.090 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — p5_retain_20261008_jitter_jitter_combined_2

- Wall duration: 51.518 s
- Received / published control frames: 2041 / 2041
- Published control FPS (ROS simulation time / active wall time): 60.000 / 54.703 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.281 / 16.622 / 25.446 / 87.383 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.090 / 0.086 / 0.113 / 0.858 ms (steady clock)

#### Controller — p5_retain_20261008_jitter_jitter_combined_2

- Wall duration: 51.627 s
- Timer callbacks / command frames / guidance frames: 2042 / 2041 / 1620
- Command FPS (ROS simulation time / active wall time): 60.000 / 54.702 Hz
- Guidance FPS (active wall time): 47.134 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.358 / 0.214 / 2.179 / 3.927 ms
- Command interval mean / P50 / P95 / max: 18.281 / 16.626 / 25.563 / 87.560 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 43.868 / 33.333 / 83.333 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.745 / 0.619 / 8.191 / 14.285 ms (steady clock)
- DDS publish call mean / P95 / max: 0.050 / 0.066 / 2.464 ms (steady clock)
