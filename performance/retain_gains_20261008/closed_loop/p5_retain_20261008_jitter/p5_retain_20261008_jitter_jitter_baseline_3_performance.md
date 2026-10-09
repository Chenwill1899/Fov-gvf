
### Run p5_retain_20261008_jitter_jitter_baseline_3

- Started: 2026-10-08 10:47:19 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_retain_20261008_jitter_jitter_baseline_3

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 36.566 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 55.872 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.743 / 16.637 / 18.718 / 78.713 ms
- Distinct applied command values: 1496
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.838 / 16.870 / 34.311 / 1990.781 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_retain_20261008_jitter_jitter_baseline_3

- Wall duration: 50.536 s
- Timer callbacks / command frames / guidance frames: 2042 / 2041 / 1620
- Command FPS (ROS simulation time / active wall time): 60.000 / 56.368 Hz
- Guidance FPS (active wall time): 48.763 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.363 / 0.212 / 2.146 / 3.809 ms
- Command interval mean / P50 / P95 / max: 17.741 / 16.172 / 25.708 / 85.539 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 44.033 / 33.333 / 100.000 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.696 / 0.640 / 8.797 / 14.291 ms (steady clock)
- DDS publish call mean / P95 / max: 0.047 / 0.060 / 1.250 ms (steady clock)

#### Command bridge — p5_retain_20261008_jitter_jitter_baseline_3

- Wall duration: 50.425 s
- Received / published control frames: 2041 / 2041
- Published control FPS (ROS simulation time / active wall time): 60.000 / 56.368 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.741 / 16.203 / 25.577 / 86.417 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.087 / 0.084 / 0.106 / 0.548 ms (steady clock)
