
### Run p5_retain_20261008_jitter_jitter_v4_1

- Started: 2026-10-08 10:17:54 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_retain_20261008_jitter_jitter_v4_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 36.874 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 55.405 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.889 / 16.675 / 18.967 / 79.950 ms
- Distinct applied command values: 1567
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.950 / 16.807 / 19.589 / 2019.880 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_retain_20261008_jitter_jitter_v4_1

- Wall duration: 50.832 s
- Timer callbacks / command frames / guidance frames: 2042 / 2041 / 1620
- Command FPS (ROS simulation time / active wall time): 60.000 / 55.915 Hz
- Guidance FPS (active wall time): 48.438 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.356 / 0.211 / 2.185 / 4.249 ms
- Command interval mean / P50 / P95 / max: 17.884 / 16.115 / 24.858 / 86.694 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 40.072 / 33.333 / 83.333 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.752 / 1.857 / 7.641 / 12.931 ms (steady clock)
- DDS publish call mean / P95 / max: 0.049 / 0.068 / 0.765 ms (steady clock)

#### Command bridge — p5_retain_20261008_jitter_jitter_v4_1

- Wall duration: 50.721 s
- Received / published control frames: 2041 / 2041
- Published control FPS (ROS simulation time / active wall time): 60.000 / 55.915 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.884 / 16.154 / 24.694 / 86.630 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.087 / 0.083 / 0.107 / 0.904 ms (steady clock)
