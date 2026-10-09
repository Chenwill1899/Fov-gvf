
### Run p5_refine_execution_jitter_20261007_jitter_after_2

- Started: 2026-10-07 22:16:13 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_refine_execution_jitter_20261007_jitter_after_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 37.141 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 55.007 Hz
- Isaac frame interval mean / P50 / P95 / max: 18.013 / 16.843 / 18.989 / 80.048 ms
- Distinct applied command values: 1524
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.554 / 17.077 / 34.029 / 1991.658 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_refine_execution_jitter_20261007_jitter_after_2

- Wall duration: 51.403 s
- Timer callbacks / command frames / guidance frames: 2042 / 2041 / 1620
- Command FPS (ROS simulation time / active wall time): 60.000 / 55.518 Hz
- Guidance FPS (active wall time): 48.392 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.327 / 0.206 / 2.022 / 3.603 ms
- Command interval mean / P50 / P95 / max: 18.012 / 16.671 / 25.190 / 87.162 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 41.564 / 33.333 / 83.333 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.689 / 0.486 / 7.905 / 13.775 ms (steady clock)
- DDS publish call mean / P95 / max: 0.046 / 0.057 / 1.581 ms (steady clock)

#### Command bridge — p5_refine_execution_jitter_20261007_jitter_after_2

- Wall duration: 51.292 s
- Received / published control frames: 2041 / 2041
- Published control FPS (ROS simulation time / active wall time): 60.000 / 55.518 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.012 / 16.690 / 25.177 / 87.237 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.083 / 0.081 / 0.104 / 0.428 ms (steady clock)
