
### Run p5_manual_continuous_explore_20261007_jitter_jitter_after_1

- Started: 2026-10-07 22:59:40 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_manual_continuous_explore_20261007_jitter_jitter_after_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 36.682 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 55.695 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.796 / 16.670 / 18.941 / 79.539 ms
- Distinct applied command values: 1542
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.190 / 16.878 / 33.041 / 2049.387 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — p5_manual_continuous_explore_20261007_jitter_jitter_after_1

- Wall duration: 50.165 s
- Received / published control frames: 2040 / 2040
- Published control FPS (ROS simulation time / active wall time): 59.971 / 56.163 Hz
- Command stamp interval mean / P50 / P95 / max: 16.675 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.805 / 16.830 / 24.655 / 84.705 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.086 / 0.083 / 0.108 / 1.180 ms (steady clock)

#### Controller — p5_manual_continuous_explore_20261007_jitter_jitter_after_1

- Wall duration: 50.610 s
- Timer callbacks / command frames / guidance frames: 2041 / 2040 / 1619
- Command FPS (ROS simulation time / active wall time): 59.971 / 56.163 Hz
- Guidance FPS (active wall time): 48.755 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.675 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.283 / 0.207 / 0.291 / 3.823 ms
- Command interval mean / P50 / P95 / max: 17.805 / 16.806 / 24.736 / 84.650 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 41.734 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.731 / 2.538 / 7.504 / 14.114 ms (steady clock)
- DDS publish call mean / P95 / max: 0.047 / 0.063 / 2.141 ms (steady clock)
