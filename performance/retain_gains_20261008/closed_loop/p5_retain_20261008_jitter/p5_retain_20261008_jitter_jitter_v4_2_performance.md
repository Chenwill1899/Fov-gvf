
### Run p5_retain_20261008_jitter_jitter_v4_2

- Started: 2026-10-08 10:25:16 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_retain_20261008_jitter_jitter_v4_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 36.870 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 55.410 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.888 / 16.681 / 18.895 / 79.807 ms
- Distinct applied command values: 1564
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.025 / 16.832 / 19.557 / 2198.326 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_retain_20261008_jitter_jitter_v4_2

- Wall duration: 50.801 s
- Timer callbacks / command frames / guidance frames: 2040 / 2039 / 1619
- Command FPS (ROS simulation time / active wall time): 59.941 / 55.848 Hz
- Guidance FPS (active wall time): 48.231 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.683 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.381 / 0.211 / 2.284 / 3.695 ms
- Command interval mean / P50 / P95 / max: 17.906 / 16.455 / 24.483 / 86.721 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 38.429 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.848 / 2.712 / 7.329 / 13.888 ms (steady clock)
- DDS publish call mean / P95 / max: 0.052 / 0.072 / 2.240 ms (steady clock)

#### Command bridge — p5_retain_20261008_jitter_jitter_v4_2

- Wall duration: 50.690 s
- Received / published control frames: 2039 / 2039
- Published control FPS (ROS simulation time / active wall time): 59.941 / 55.848 Hz
- Command stamp interval mean / P50 / P95 / max: 16.683 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.906 / 16.444 / 24.195 / 86.570 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.087 / 0.083 / 0.109 / 1.050 ms (steady clock)
