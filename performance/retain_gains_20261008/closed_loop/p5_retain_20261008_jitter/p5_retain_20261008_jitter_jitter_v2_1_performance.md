
### Run p5_retain_20261008_jitter_jitter_v2_1

- Started: 2026-10-08 10:10:32 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_retain_20261008_jitter_jitter_v2_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 36.831 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 55.470 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.867 / 16.611 / 18.683 / 81.488 ms
- Distinct applied command values: 1514
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.758 / 16.817 / 34.170 / 2074.531 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_retain_20261008_jitter_jitter_v2_1

- Wall duration: 50.707 s
- Timer callbacks / command frames / guidance frames: 2042 / 2041 / 1620
- Command FPS (ROS simulation time / active wall time): 60.000 / 55.968 Hz
- Guidance FPS (active wall time): 48.527 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.335 / 0.213 / 2.060 / 4.190 ms
- Command interval mean / P50 / P95 / max: 17.867 / 16.369 / 25.703 / 91.557 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 39.897 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.780 / 2.153 / 8.729 / 13.504 ms (steady clock)
- DDS publish call mean / P95 / max: 0.057 / 0.105 / 1.520 ms (steady clock)

#### Command bridge — p5_retain_20261008_jitter_jitter_v2_1

- Wall duration: 50.595 s
- Received / published control frames: 2041 / 2041
- Published control FPS (ROS simulation time / active wall time): 60.000 / 55.968 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.867 / 16.422 / 25.689 / 91.586 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.088 / 0.084 / 0.110 / 1.230 ms (steady clock)
