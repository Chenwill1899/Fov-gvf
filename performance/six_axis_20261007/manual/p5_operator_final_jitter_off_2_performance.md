
### Run p5_operator_final_jitter_off_2

- Started: 2026-10-07 17:00:48 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_operator_final_jitter_off_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 36.596 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 55.826 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.753 / 16.825 / 18.933 / 81.276 ms
- Distinct applied command values: 1377
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 24.764 / 17.320 / 35.005 / 2189.245 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_operator_final_jitter_off_2

- Wall duration: 50.897 s
- Timer callbacks / command frames / guidance frames: 2042 / 2041 / 1620
- Command FPS (ROS simulation time / active wall time): 60.000 / 56.352 Hz
- Guidance FPS (active wall time): 48.777 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.370 / 0.212 / 2.152 / 3.298 ms
- Command interval mean / P50 / P95 / max: 17.746 / 16.742 / 23.768 / 82.928 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 39.311 / 33.333 / 66.667 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.797 / 3.093 / 6.788 / 13.188 ms (steady clock)
- DDS publish call mean / P95 / max: 0.050 / 0.070 / 0.869 ms (steady clock)

#### Command bridge — p5_operator_final_jitter_off_2

- Wall duration: 50.888 s
- Received / published control frames: 2041 / 2041
- Published control FPS (ROS simulation time / active wall time): 60.000 / 56.352 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.746 / 16.755 / 23.613 / 82.796 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.086 / 0.084 / 0.109 / 0.451 ms (steady clock)
