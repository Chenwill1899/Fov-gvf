
### Run p5_operator_final_jitter_off_1

- Started: 2026-10-07 16:58:13 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_operator_final_jitter_off_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 36.517 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 55.947 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.715 / 16.729 / 19.109 / 79.173 ms
- Distinct applied command values: 1378
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 24.784 / 17.253 / 34.831 / 2059.715 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — p5_operator_final_jitter_off_1

- Wall duration: 50.575 s
- Received / published control frames: 2041 / 2041
- Published control FPS (ROS simulation time / active wall time): 60.000 / 56.453 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.714 / 16.339 / 24.325 / 83.527 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.085 / 0.083 / 0.106 / 0.341 ms (steady clock)

#### Controller — p5_operator_final_jitter_off_1

- Wall duration: 50.585 s
- Timer callbacks / command frames / guidance frames: 2042 / 2041 / 1620
- Command FPS (ROS simulation time / active wall time): 60.000 / 56.453 Hz
- Guidance FPS (active wall time): 48.687 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.274 / 0.208 / 0.287 / 3.737 ms
- Command interval mean / P50 / P95 / max: 17.714 / 16.357 / 24.485 / 83.659 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 36.368 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.754 / 2.737 / 7.334 / 13.528 ms (steady clock)
- DDS publish call mean / P95 / max: 0.053 / 0.087 / 1.761 ms (steady clock)
