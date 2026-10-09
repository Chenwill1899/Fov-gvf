
### Run p5_manual_continuous_explore_20261007_jitter_jitter_before_2

- Started: 2026-10-07 23:04:29 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_manual_continuous_explore_20261007_jitter_jitter_before_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 36.366 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 56.179 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.639 / 16.536 / 18.716 / 78.838 ms
- Distinct applied command values: 1509
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.545 / 16.711 / 34.178 / 2043.686 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_manual_continuous_explore_20261007_jitter_jitter_before_2

- Wall duration: 50.638 s
- Timer callbacks / command frames / guidance frames: 2041 / 2040 / 1619
- Command FPS (ROS simulation time / active wall time): 59.971 / 56.668 Hz
- Guidance FPS (active wall time): 48.849 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.675 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.335 / 0.208 / 2.079 / 3.618 ms
- Command interval mean / P50 / P95 / max: 17.647 / 16.289 / 25.264 / 88.232 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 45.213 / 50.000 / 83.333 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.653 / 0.397 / 8.329 / 14.265 ms (steady clock)
- DDS publish call mean / P95 / max: 0.046 / 0.057 / 1.390 ms (steady clock)

#### Command bridge — p5_manual_continuous_explore_20261007_jitter_jitter_before_2

- Wall duration: 50.527 s
- Received / published control frames: 2040 / 2040
- Published control FPS (ROS simulation time / active wall time): 59.971 / 56.669 Hz
- Command stamp interval mean / P50 / P95 / max: 16.675 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.646 / 16.291 / 25.216 / 87.978 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.083 / 0.082 / 0.105 / 0.473 ms (steady clock)
