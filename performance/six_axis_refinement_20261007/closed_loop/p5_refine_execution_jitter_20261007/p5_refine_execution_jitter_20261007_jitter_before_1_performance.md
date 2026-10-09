
### Run p5_refine_execution_jitter_20261007_jitter_before_1

- Started: 2026-10-07 22:12:59 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_refine_execution_jitter_20261007_jitter_before_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 36.615 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 55.796 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.760 / 16.858 / 19.036 / 78.745 ms
- Distinct applied command values: 1320
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 25.944 / 17.517 / 35.424 / 2076.316 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_refine_execution_jitter_20261007_jitter_before_1

- Wall duration: 50.505 s
- Timer callbacks / command frames / guidance frames: 2042 / 2041 / 1620
- Command FPS (ROS simulation time / active wall time): 60.000 / 56.308 Hz
- Guidance FPS (active wall time): 48.745 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.299 / 0.210 / 0.525 / 3.423 ms
- Command interval mean / P50 / P95 / max: 17.759 / 16.731 / 25.479 / 87.303 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 39.290 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.655 / 1.637 / 8.383 / 13.819 ms (steady clock)
- DDS publish call mean / P95 / max: 0.053 / 0.089 / 1.833 ms (steady clock)

#### Command bridge — p5_refine_execution_jitter_20261007_jitter_before_1

- Wall duration: 50.396 s
- Received / published control frames: 2041 / 2041
- Published control FPS (ROS simulation time / active wall time): 60.000 / 56.308 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.759 / 16.691 / 25.374 / 87.149 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.086 / 0.084 / 0.108 / 0.488 ms (steady clock)
