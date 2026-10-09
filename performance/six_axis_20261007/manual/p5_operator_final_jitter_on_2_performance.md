
### Run p5_operator_final_jitter_on_2

- Started: 2026-10-07 16:59:57 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_operator_final_jitter_on_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 36.906 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 55.358 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.904 / 16.788 / 18.973 / 80.373 ms
- Distinct applied command values: 1336
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 25.849 / 17.423 / 35.654 / 2093.515 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_operator_final_jitter_on_2

- Wall duration: 50.909 s
- Timer callbacks / command frames / guidance frames: 2042 / 2042 / 1620
- Command FPS (ROS simulation time / active wall time): 60.000 / 55.321 Hz
- Guidance FPS (active wall time): 48.130 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.297 / 0.208 / 0.333 / 3.584 ms
- Command interval mean / P50 / P95 / max: 18.076 / 16.368 / 24.527 / 368.534 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 40.556 / 33.333 / 83.333 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.721 / 2.383 / 7.614 / 14.109 ms (steady clock)
- DDS publish call mean / P95 / max: 0.050 / 0.066 / 1.369 ms (steady clock)

#### Command bridge — p5_operator_final_jitter_on_2

- Wall duration: 50.804 s
- Received / published control frames: 2042 / 2042
- Published control FPS (ROS simulation time / active wall time): 60.000 / 55.321 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.076 / 16.374 / 24.304 / 368.752 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.085 / 0.083 / 0.107 / 0.959 ms (steady clock)
