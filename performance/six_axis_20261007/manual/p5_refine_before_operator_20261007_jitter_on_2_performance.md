
### Run p5_refine_before_operator_20261007_jitter_on_2

- Started: 2026-10-07 21:29:45 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_refine_before_operator_20261007_jitter_on_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 37.802 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 54.044 Hz
- Isaac frame interval mean / P50 / P95 / max: 18.345 / 16.912 / 19.399 / 81.118 ms
- Distinct applied command values: 1304
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 27.072 / 17.665 / 36.262 / 2273.659 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — p5_refine_before_operator_20261007_jitter_on_2

- Wall duration: 52.167 s
- Received / published control frames: 2040 / 2040
- Published control FPS (ROS simulation time / active wall time): 59.971 / 54.486 Hz
- Command stamp interval mean / P50 / P95 / max: 16.675 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.353 / 16.702 / 26.089 / 89.537 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.092 / 0.086 / 0.117 / 1.271 ms (steady clock)

#### Controller — p5_refine_before_operator_20261007_jitter_on_2

- Wall duration: 52.176 s
- Timer callbacks / command frames / guidance frames: 2041 / 2040 / 1619
- Command FPS (ROS simulation time / active wall time): 59.971 / 54.486 Hz
- Guidance FPS (active wall time): 47.059 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.675 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.313 / 0.211 / 0.791 / 4.524 ms
- Command interval mean / P50 / P95 / max: 18.353 / 16.714 / 26.111 / 89.368 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 45.512 / 33.333 / 83.333 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.795 / 0.483 / 8.430 / 16.532 ms (steady clock)
- DDS publish call mean / P95 / max: 0.051 / 0.070 / 1.803 ms (steady clock)
