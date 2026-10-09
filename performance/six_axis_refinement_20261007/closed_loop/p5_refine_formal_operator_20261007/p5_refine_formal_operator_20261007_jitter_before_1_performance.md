
### Run p5_refine_formal_operator_20261007_jitter_before_1

- Started: 2026-10-07 21:42:06 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_refine_formal_operator_20261007_jitter_before_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 36.646 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 55.749 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.776 / 16.938 / 19.005 / 78.870 ms
- Distinct applied command values: 1381
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 24.808 / 17.491 / 35.163 / 2130.492 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_refine_formal_operator_20261007_jitter_before_1

- Wall duration: 50.608 s
- Timer callbacks / command frames / guidance frames: 2042 / 2041 / 1620
- Command FPS (ROS simulation time / active wall time): 60.000 / 56.256 Hz
- Guidance FPS (active wall time): 48.553 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.314 / 0.208 / 1.708 / 3.986 ms
- Command interval mean / P50 / P95 / max: 17.776 / 16.726 / 24.862 / 86.803 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 36.019 / 33.333 / 66.667 / 83.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.773 / 2.523 / 7.737 / 13.801 ms (steady clock)
- DDS publish call mean / P95 / max: 0.054 / 0.103 / 1.017 ms (steady clock)

#### Command bridge — p5_refine_formal_operator_20261007_jitter_before_1

- Wall duration: 50.496 s
- Received / published control frames: 2041 / 2041
- Published control FPS (ROS simulation time / active wall time): 60.000 / 56.256 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.776 / 16.721 / 24.753 / 86.783 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.088 / 0.086 / 0.111 / 0.370 ms (steady clock)
