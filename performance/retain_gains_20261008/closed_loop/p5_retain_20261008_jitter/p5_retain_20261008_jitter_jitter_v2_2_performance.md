
### Run p5_retain_20261008_jitter_jitter_v2_2

- Started: 2026-10-08 10:21:34 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_retain_20261008_jitter_jitter_v2_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 36.573 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 55.860 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.711 / 16.753 / 18.875 / 79.781 ms
- Distinct applied command values: 1512
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.501 / 16.954 / 34.199 / 2093.305 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — p5_retain_20261008_jitter_jitter_v2_2

- Wall duration: 50.402 s
- Received / published control frames: 2041 / 2041
- Published control FPS (ROS simulation time / active wall time): 60.000 / 56.456 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.713 / 16.530 / 25.770 / 86.970 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.084 / 0.083 / 0.104 / 0.528 ms (steady clock)

#### Controller — p5_retain_20261008_jitter_jitter_v2_2

- Wall duration: 50.516 s
- Timer callbacks / command frames / guidance frames: 2042 / 2041 / 1620
- Command FPS (ROS simulation time / active wall time): 60.000 / 56.456 Hz
- Guidance FPS (active wall time): 48.916 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.353 / 0.211 / 2.135 / 4.066 ms
- Command interval mean / P50 / P95 / max: 17.713 / 16.522 / 25.798 / 86.904 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 47.726 / 50.000 / 83.333 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.647 / 0.390 / 8.786 / 13.393 ms (steady clock)
- DDS publish call mean / P95 / max: 0.049 / 0.054 / 2.223 ms (steady clock)
