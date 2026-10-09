
### Run p5_refine_before_operator_20261007_jitter_off_2

- Started: 2026-10-07 21:30:39 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_refine_before_operator_20261007_jitter_off_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 38.373 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 53.240 Hz
- Isaac frame interval mean / P50 / P95 / max: 18.615 / 16.906 / 20.152 / 82.244 ms
- Distinct applied command values: 1354
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 26.544 / 17.608 / 37.309 / 2204.364 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_refine_before_operator_20261007_jitter_off_2

- Wall duration: 52.426 s
- Timer callbacks / command frames / guidance frames: 2041 / 2040 / 1620
- Command FPS (ROS simulation time / active wall time): 59.971 / 53.698 Hz
- Guidance FPS (active wall time): 46.406 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.675 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.301 / 0.212 / 0.591 / 3.494 ms
- Command interval mean / P50 / P95 / max: 18.623 / 17.118 / 25.003 / 87.365 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 39.527 / 33.333 / 66.667 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.815 / 2.486 / 7.463 / 14.045 ms (steady clock)
- DDS publish call mean / P95 / max: 0.053 / 0.077 / 1.256 ms (steady clock)

#### Command bridge — p5_refine_before_operator_20261007_jitter_off_2

- Wall duration: 52.315 s
- Received / published control frames: 2040 / 2040
- Published control FPS (ROS simulation time / active wall time): 59.971 / 53.699 Hz
- Command stamp interval mean / P50 / P95 / max: 16.675 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.622 / 17.093 / 24.805 / 87.190 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.091 / 0.086 / 0.113 / 1.719 ms (steady clock)
