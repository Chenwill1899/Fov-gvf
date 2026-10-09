
### Run p5_refine_formal_operator_20261007_jitter_before_2

- Started: 2026-10-07 21:48:45 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_refine_formal_operator_20261007_jitter_before_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 38.044 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 53.701 Hz
- Isaac frame interval mean / P50 / P95 / max: 18.462 / 16.851 / 19.127 / 79.850 ms
- Distinct applied command values: 1320
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 27.027 / 17.536 / 35.912 / 2195.566 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_refine_formal_operator_20261007_jitter_before_2

- Wall duration: 52.014 s
- Timer callbacks / command frames / guidance frames: 2041 / 2040 / 1619
- Command FPS (ROS simulation time / active wall time): 59.971 / 54.135 Hz
- Guidance FPS (active wall time): 46.635 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.675 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.326 / 0.211 / 1.998 / 4.332 ms
- Command interval mean / P50 / P95 / max: 18.472 / 16.781 / 26.164 / 87.382 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 42.094 / 33.333 / 83.333 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.813 / 2.248 / 8.563 / 13.595 ms (steady clock)
- DDS publish call mean / P95 / max: 0.048 / 0.063 / 0.908 ms (steady clock)

#### Command bridge — p5_refine_formal_operator_20261007_jitter_before_2

- Wall duration: 51.903 s
- Received / published control frames: 2040 / 2040
- Published control FPS (ROS simulation time / active wall time): 59.971 / 54.135 Hz
- Command stamp interval mean / P50 / P95 / max: 16.675 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.472 / 16.799 / 26.048 / 87.333 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.090 / 0.086 / 0.112 / 0.886 ms (steady clock)
