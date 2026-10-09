
### Run p5_retain_20261008_jitter_jitter_combined_1

- Started: 2026-10-08 10:14:10 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_retain_20261008_jitter_jitter_combined_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 37.923 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 53.872 Hz
- Isaac frame interval mean / P50 / P95 / max: 18.409 / 16.983 / 19.179 / 80.473 ms
- Distinct applied command values: 1535
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 23.116 / 17.165 / 34.156 / 2172.338 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_retain_20261008_jitter_jitter_combined_1

- Wall duration: 51.926 s
- Timer callbacks / command frames / guidance frames: 2042 / 2041 / 1620
- Command FPS (ROS simulation time / active wall time): 60.000 / 54.322 Hz
- Guidance FPS (active wall time): 46.918 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.371 / 0.212 / 2.229 / 3.381 ms
- Command interval mean / P50 / P95 / max: 18.409 / 17.164 / 24.800 / 85.751 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 40.309 / 33.333 / 83.333 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.758 / 2.487 / 7.557 / 13.331 ms (steady clock)
- DDS publish call mean / P95 / max: 0.052 / 0.072 / 1.109 ms (steady clock)

#### Command bridge — p5_retain_20261008_jitter_jitter_combined_1

- Wall duration: 51.816 s
- Received / published control frames: 2041 / 2041
- Published control FPS (ROS simulation time / active wall time): 60.000 / 54.323 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.409 / 17.139 / 24.792 / 85.691 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.089 / 0.086 / 0.111 / 0.805 ms (steady clock)
