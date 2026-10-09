
### Run candidate_v21_sweep_ego1p3_1

- Started: 2026-10-07 01:35:41 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — candidate_v21_sweep_ego1p3_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 42.417 / 40.017 s
- Applied simulation control frames: 2403
- Runtime FPS (simulation time / wall time): 60.050 / 56.651 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.520 / 15.996 / 18.060 / 79.798 ms
- Distinct applied command values: 1923
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 20.871 / 16.478 / 33.053 / 1004.527 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — candidate_v21_sweep_ego1p3_1

- Wall duration: 56.475 s
- Timer callbacks / command frames / guidance frames: 2400 / 2399 / 2218
- Command FPS (ROS simulation time / active wall time): 59.950 / 57.034 Hz
- Guidance FPS (active wall time): 56.684 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.681 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.379 / 0.222 / 1.562 / 7.813 ms
- Command interval mean / P50 / P95 / max: 17.533 / 16.158 / 22.758 / 83.667 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 41.847 / 33.333 / 83.333 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.522 / 2.381 / 7.090 / 13.238 ms (steady clock)
- DDS publish call mean / P95 / max: 0.054 / 0.082 / 1.234 ms (steady clock)

#### Command bridge — candidate_v21_sweep_ego1p3_1

- Wall duration: 56.369 s
- Received / published control frames: 2399 / 2399
- Published control FPS (ROS simulation time / active wall time): 59.950 / 57.034 Hz
- Command stamp interval mean / P50 / P95 / max: 16.681 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.533 / 16.169 / 22.586 / 83.507 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.090 / 0.087 / 0.116 / 0.644 ms (steady clock)
