
### Run p5_operator_v2_jitter_off_2

- Started: 2026-10-07 17:22:10 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_operator_v2_jitter_off_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 37.137 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 55.012 Hz
- Isaac frame interval mean / P50 / P95 / max: 18.018 / 16.799 / 19.057 / 79.371 ms
- Distinct applied command values: 1315
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 26.425 / 17.474 / 35.198 / 2206.886 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — p5_operator_v2_jitter_off_2

- Wall duration: 50.900 s
- Received / published control frames: 2041 / 2041
- Published control FPS (ROS simulation time / active wall time): 60.000 / 55.503 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.017 / 16.357 / 26.001 / 85.458 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.089 / 0.085 / 0.111 / 1.180 ms (steady clock)

#### Controller — p5_operator_v2_jitter_off_2

- Wall duration: 51.006 s
- Timer callbacks / command frames / guidance frames: 2042 / 2041 / 1620
- Command FPS (ROS simulation time / active wall time): 60.000 / 55.502 Hz
- Guidance FPS (active wall time): 47.852 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.285 / 0.211 / 0.338 / 4.064 ms
- Command interval mean / P50 / P95 / max: 18.017 / 16.354 / 26.071 / 85.444 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 40.525 / 33.333 / 83.333 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.717 / 1.080 / 8.430 / 14.206 ms (steady clock)
- DDS publish call mean / P95 / max: 0.050 / 0.069 / 1.473 ms (steady clock)
