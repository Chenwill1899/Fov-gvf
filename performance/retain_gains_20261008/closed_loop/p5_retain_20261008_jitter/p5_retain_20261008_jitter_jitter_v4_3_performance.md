
### Run p5_retain_20261008_jitter_jitter_v4_3

- Started: 2026-10-08 10:36:18 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_retain_20261008_jitter_jitter_v4_3

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 36.997 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 55.220 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.920 / 16.660 / 18.757 / 79.758 ms
- Distinct applied command values: 1482
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 23.327 / 16.843 / 34.814 / 2125.425 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_retain_20261008_jitter_jitter_v4_3

- Wall duration: 50.914 s
- Timer callbacks / command frames / guidance frames: 2042 / 2042 / 1620
- Command FPS (ROS simulation time / active wall time): 60.000 / 55.180 Hz
- Guidance FPS (active wall time): 48.077 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.323 / 0.201 / 2.020 / 3.781 ms
- Command interval mean / P50 / P95 / max: 18.123 / 16.547 / 26.496 / 441.466 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 48.066 / 50.000 / 83.333 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.615 / 0.382 / 9.044 / 13.625 ms (steady clock)
- DDS publish call mean / P95 / max: 0.048 / 0.058 / 1.089 ms (steady clock)

#### Command bridge — p5_retain_20261008_jitter_jitter_v4_3

- Wall duration: 50.803 s
- Received / published control frames: 2042 / 2042
- Published control FPS (ROS simulation time / active wall time): 60.000 / 55.180 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.122 / 16.589 / 26.550 / 441.283 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.085 / 0.081 / 0.107 / 1.375 ms (steady clock)
