
### Run p5_operator_v2_jitter_on_1

- Started: 2026-10-07 17:20:26 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_operator_v2_jitter_on_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 36.940 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 55.306 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.923 / 16.545 / 18.984 / 79.784 ms
- Distinct applied command values: 1191
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 29.049 / 17.777 / 37.877 / 2011.277 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_operator_v2_jitter_on_1

- Wall duration: 50.809 s
- Timer callbacks / command frames / guidance frames: 2042 / 2042 / 1620
- Command FPS (ROS simulation time / active wall time): 60.000 / 55.268 Hz
- Guidance FPS (active wall time): 48.048 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.263 / 0.213 / 0.268 / 3.824 ms
- Command interval mean / P50 / P95 / max: 18.094 / 16.438 / 32.265 / 376.193 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 54.342 / 50.000 / 100.000 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.517 / 0.368 / 14.086 / 21.219 ms (steady clock)
- DDS publish call mean / P95 / max: 0.050 / 0.070 / 1.210 ms (steady clock)

#### Command bridge — p5_operator_v2_jitter_on_1

- Wall duration: 50.702 s
- Received / published control frames: 2042 / 2042
- Published control FPS (ROS simulation time / active wall time): 60.000 / 55.268 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.302 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.094 / 16.460 / 32.035 / 376.136 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.081 / 0.078 / 0.109 / 0.483 ms (steady clock)
