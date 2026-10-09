
### Run p5_refine_formal_operator_20261007_jitter_after_3

- Started: 2026-10-07 21:53:27 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_refine_formal_operator_20261007_jitter_after_3

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 37.010 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 55.201 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.958 / 16.961 / 19.218 / 79.924 ms
- Distinct applied command values: 1277
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 27.026 / 17.781 / 35.913 / 2008.735 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_refine_formal_operator_20261007_jitter_after_3

- Wall duration: 50.895 s
- Timer callbacks / command frames / guidance frames: 2042 / 2041 / 1620
- Command FPS (ROS simulation time / active wall time): 60.000 / 55.688 Hz
- Guidance FPS (active wall time): 48.282 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.310 / 0.210 / 1.573 / 3.013 ms
- Command interval mean / P50 / P95 / max: 17.957 / 16.911 / 26.705 / 106.051 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 50.628 / 50.000 / 100.000 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.623 / 0.396 / 9.318 / 19.138 ms (steady clock)
- DDS publish call mean / P95 / max: 0.052 / 0.067 / 2.925 ms (steady clock)

#### Command bridge — p5_refine_formal_operator_20261007_jitter_after_3

- Wall duration: 50.785 s
- Received / published control frames: 2041 / 2041
- Published control FPS (ROS simulation time / active wall time): 60.000 / 55.688 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.957 / 16.872 / 26.548 / 105.920 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.088 / 0.084 / 0.111 / 1.469 ms (steady clock)
