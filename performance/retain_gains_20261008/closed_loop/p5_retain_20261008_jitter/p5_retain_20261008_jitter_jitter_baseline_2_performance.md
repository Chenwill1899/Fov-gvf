
### Run p5_retain_20261008_jitter_jitter_baseline_2

- Started: 2026-10-08 10:28:56 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_retain_20261008_jitter_jitter_baseline_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 37.093 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 55.078 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.998 / 16.840 / 19.061 / 79.899 ms
- Distinct applied command values: 1575
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.931 / 17.011 / 19.433 / 2090.996 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_retain_20261008_jitter_jitter_baseline_2

- Wall duration: 51.008 s
- Timer callbacks / command frames / guidance frames: 2042 / 2041 / 1620
- Command FPS (ROS simulation time / active wall time): 60.000 / 55.585 Hz
- Guidance FPS (active wall time): 48.145 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.344 / 0.207 / 2.124 / 3.315 ms
- Command interval mean / P50 / P95 / max: 17.991 / 16.741 / 23.221 / 83.741 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 38.395 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.812 / 3.021 / 6.694 / 12.970 ms (steady clock)
- DDS publish call mean / P95 / max: 0.049 / 0.070 / 0.812 ms (steady clock)

#### Command bridge — p5_retain_20261008_jitter_jitter_baseline_2

- Wall duration: 50.898 s
- Received / published control frames: 2041 / 2041
- Published control FPS (ROS simulation time / active wall time): 60.000 / 55.585 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.991 / 16.710 / 23.206 / 83.586 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.085 / 0.083 / 0.106 / 0.790 ms (steady clock)
