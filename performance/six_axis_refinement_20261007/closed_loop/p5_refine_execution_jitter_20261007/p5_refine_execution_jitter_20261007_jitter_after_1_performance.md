
### Run p5_refine_execution_jitter_20261007_jitter_after_1

- Started: 2026-10-07 22:13:51 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_refine_execution_jitter_20261007_jitter_after_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 36.980 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 55.247 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.938 / 16.849 / 19.160 / 80.582 ms
- Distinct applied command values: 1570
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.005 / 17.022 / 19.796 / 2085.745 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — p5_refine_execution_jitter_20261007_jitter_after_1

- Wall duration: 50.921 s
- Received / published control frames: 2041 / 2041
- Published control FPS (ROS simulation time / active wall time): 60.000 / 55.752 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.936 / 16.365 / 24.952 / 85.685 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.092 / 0.085 / 0.118 / 1.631 ms (steady clock)

#### Controller — p5_refine_execution_jitter_20261007_jitter_after_1

- Wall duration: 51.035 s
- Timer callbacks / command frames / guidance frames: 2042 / 2041 / 1620
- Command FPS (ROS simulation time / active wall time): 60.000 / 55.752 Hz
- Guidance FPS (active wall time): 48.238 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.411 / 0.214 / 2.289 / 4.360 ms
- Command interval mean / P50 / P95 / max: 17.937 / 16.329 / 25.192 / 85.683 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 39.105 / 33.333 / 66.667 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.836 / 2.330 / 7.877 / 13.952 ms (steady clock)
- DDS publish call mean / P95 / max: 0.052 / 0.079 / 1.149 ms (steady clock)
