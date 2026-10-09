
### Run p5_refine_before_operator_20261007_jitter_off_1

- Started: 2026-10-07 21:26:30 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_refine_before_operator_20261007_jitter_off_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 37.893 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 53.915 Hz
- Isaac frame interval mean / P50 / P95 / max: 18.390 / 16.742 / 20.251 / 82.385 ms
- Distinct applied command values: 1348
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 26.364 / 17.506 / 37.449 / 2071.437 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — p5_refine_before_operator_20261007_jitter_off_1

- Wall duration: 52.392 s
- Received / published control frames: 2041 / 2041
- Published control FPS (ROS simulation time / active wall time): 60.000 / 54.375 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.391 / 16.884 / 24.306 / 88.468 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.085 / 0.082 / 0.109 / 1.085 ms (steady clock)

#### Controller — p5_refine_before_operator_20261007_jitter_off_1

- Wall duration: 52.402 s
- Timer callbacks / command frames / guidance frames: 2042 / 2041 / 1620
- Command FPS (ROS simulation time / active wall time): 60.000 / 54.374 Hz
- Guidance FPS (active wall time): 46.939 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.312 / 0.206 / 1.943 / 3.260 ms
- Command interval mean / P50 / P95 / max: 18.391 / 16.930 / 24.315 / 88.696 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 40.216 / 33.333 / 83.333 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.839 / 3.068 / 6.736 / 14.400 ms (steady clock)
- DDS publish call mean / P95 / max: 0.051 / 0.074 / 1.041 ms (steady clock)
