
### Run p5_operator_v2_jitter_on_2

- Started: 2026-10-07 17:21:18 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_operator_v2_jitter_on_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 37.306 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 54.763 Hz
- Isaac frame interval mean / P50 / P95 / max: 18.103 / 16.761 / 19.075 / 79.282 ms
- Distinct applied command values: 1341
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 25.962 / 17.393 / 35.459 / 2040.172 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — p5_operator_v2_jitter_on_2

- Wall duration: 51.400 s
- Received / published control frames: 2041 / 2041
- Published control FPS (ROS simulation time / active wall time): 60.000 / 55.241 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.102 / 16.543 / 24.158 / 84.831 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.084 / 0.081 / 0.104 / 1.399 ms (steady clock)

#### Controller — p5_operator_v2_jitter_on_2

- Wall duration: 51.508 s
- Timer callbacks / command frames / guidance frames: 2042 / 2041 / 1620
- Command FPS (ROS simulation time / active wall time): 60.000 / 55.241 Hz
- Guidance FPS (active wall time): 47.749 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.332 / 0.208 / 2.029 / 4.221 ms
- Command interval mean / P50 / P95 / max: 18.103 / 16.581 / 24.255 / 84.863 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 41.944 / 33.333 / 83.333 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.727 / 2.279 / 7.994 / 13.585 ms (steady clock)
- DDS publish call mean / P95 / max: 0.049 / 0.068 / 1.180 ms (steady clock)
