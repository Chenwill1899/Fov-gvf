
### Run p5_retain_20261008_jitter_jitter_v2_3

- Started: 2026-10-08 10:43:37 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_retain_20261008_jitter_jitter_v2_3

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 36.079 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 56.626 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.501 / 16.534 / 18.768 / 78.402 ms
- Distinct applied command values: 1541
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.855 / 16.703 / 32.166 / 2049.790 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_retain_20261008_jitter_jitter_v2_3

- Wall duration: 50.446 s
- Timer callbacks / command frames / guidance frames: 2042 / 2041 / 1620
- Command FPS (ROS simulation time / active wall time): 60.000 / 57.142 Hz
- Guidance FPS (active wall time): 49.366 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.282 / 0.209 / 0.298 / 4.388 ms
- Command interval mean / P50 / P95 / max: 17.500 / 16.189 / 25.052 / 85.116 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 37.428 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.674 / 2.266 / 8.196 / 13.587 ms (steady clock)
- DDS publish call mean / P95 / max: 0.053 / 0.083 / 1.515 ms (steady clock)

#### Command bridge — p5_retain_20261008_jitter_jitter_v2_3

- Wall duration: 50.334 s
- Received / published control frames: 2041 / 2041
- Published control FPS (ROS simulation time / active wall time): 60.000 / 57.142 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.500 / 16.166 / 24.926 / 84.969 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.087 / 0.083 / 0.106 / 1.938 ms (steady clock)
