
### Run p5_retain_20261008_jitter_jitter_baseline_1

- Started: 2026-10-08 10:09:39 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_retain_20261008_jitter_jitter_baseline_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 37.476 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 54.514 Hz
- Isaac frame interval mean / P50 / P95 / max: 18.183 / 16.868 / 19.179 / 79.374 ms
- Distinct applied command values: 1580
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.217 / 17.041 / 19.662 / 2065.835 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_retain_20261008_jitter_jitter_baseline_1

- Wall duration: 51.584 s
- Timer callbacks / command frames / guidance frames: 2042 / 2041 / 1620
- Command FPS (ROS simulation time / active wall time): 60.000 / 55.000 Hz
- Guidance FPS (active wall time): 47.425 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.361 / 0.211 / 2.095 / 3.765 ms
- Command interval mean / P50 / P95 / max: 18.182 / 16.709 / 24.256 / 84.928 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 40.957 / 33.333 / 83.333 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.782 / 2.570 / 6.853 / 13.687 ms (steady clock)
- DDS publish call mean / P95 / max: 0.052 / 0.070 / 1.783 ms (steady clock)

#### Command bridge — p5_retain_20261008_jitter_jitter_baseline_1

- Wall duration: 51.477 s
- Received / published control frames: 2041 / 2041
- Published control FPS (ROS simulation time / active wall time): 60.000 / 55.000 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.182 / 16.698 / 24.181 / 84.420 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.086 / 0.084 / 0.108 / 0.293 ms (steady clock)
