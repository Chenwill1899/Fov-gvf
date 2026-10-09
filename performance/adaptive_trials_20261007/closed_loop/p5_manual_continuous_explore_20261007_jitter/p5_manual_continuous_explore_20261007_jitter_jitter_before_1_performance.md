
### Run p5_manual_continuous_explore_20261007_jitter_jitter_before_1

- Started: 2026-10-07 22:58:48 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_manual_continuous_explore_20261007_jitter_jitter_before_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 36.826 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 55.476 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.867 / 16.632 / 18.819 / 79.632 ms
- Distinct applied command values: 1569
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.006 / 16.793 / 19.683 / 2049.893 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_manual_continuous_explore_20261007_jitter_jitter_before_1

- Wall duration: 50.795 s
- Timer callbacks / command frames / guidance frames: 2042 / 2041 / 1620
- Command FPS (ROS simulation time / active wall time): 60.000 / 55.995 Hz
- Guidance FPS (active wall time): 48.237 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.357 / 0.207 / 2.165 / 3.533 ms
- Command interval mean / P50 / P95 / max: 17.859 / 16.308 / 24.326 / 86.990 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 38.302 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.740 / 2.406 / 7.389 / 13.309 ms (steady clock)
- DDS publish call mean / P95 / max: 0.046 / 0.063 / 1.182 ms (steady clock)

#### Command bridge — p5_manual_continuous_explore_20261007_jitter_jitter_before_1

- Wall duration: 50.687 s
- Received / published control frames: 2041 / 2041
- Published control FPS (ROS simulation time / active wall time): 60.000 / 55.995 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.859 / 16.328 / 24.274 / 86.824 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.084 / 0.082 / 0.107 / 0.246 ms (steady clock)
