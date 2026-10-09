
### Run p5_retain_20261008_jitter_jitter_combined_3

- Started: 2026-10-08 10:39:58 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_retain_20261008_jitter_jitter_combined_3

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 36.479 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 56.005 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.694 / 16.655 / 18.792 / 79.684 ms
- Distinct applied command values: 1584
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.496 / 16.810 / 19.043 / 2126.736 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_retain_20261008_jitter_jitter_combined_3

- Wall duration: 50.417 s
- Timer callbacks / command frames / guidance frames: 2042 / 2041 / 1620
- Command FPS (ROS simulation time / active wall time): 60.000 / 56.535 Hz
- Guidance FPS (active wall time): 48.927 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.364 / 0.213 / 2.110 / 3.031 ms
- Command interval mean / P50 / P95 / max: 17.688 / 16.719 / 22.274 / 83.484 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 35.350 / 33.333 / 50.000 / 83.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.804 / 3.053 / 6.105 / 13.446 ms (steady clock)
- DDS publish call mean / P95 / max: 0.054 / 0.099 / 1.728 ms (steady clock)

#### Command bridge — p5_retain_20261008_jitter_jitter_combined_3

- Wall duration: 50.306 s
- Received / published control frames: 2041 / 2041
- Published control FPS (ROS simulation time / active wall time): 60.000 / 56.535 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.688 / 16.738 / 22.177 / 82.897 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.086 / 0.083 / 0.109 / 1.036 ms (steady clock)
