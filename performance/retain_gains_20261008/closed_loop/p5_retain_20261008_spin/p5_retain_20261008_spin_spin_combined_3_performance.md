
### Run p5_retain_20261008_spin_spin_combined_3

- Started: 2026-10-08 10:41:19 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_retain_20261008_spin_spin_combined_3

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 65.901 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 58.315 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.010 / 15.627 / 17.687 / 78.458 ms
- Distinct applied command values: 3134
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 19.579 / 15.833 / 31.614 / 4190.656 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_retain_20261008_spin_spin_combined_3

- Wall duration: 79.625 s
- Timer callbacks / command frames / guidance frames: 3842 / 3841 / 3299
- Command FPS (ROS simulation time / active wall time): 60.000 / 58.791 Hz
- Guidance FPS (active wall time): 54.539 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 1.095 / 0.417 / 4.225 / 22.763 ms
- Command interval mean / P50 / P95 / max: 17.009 / 15.935 / 22.508 / 86.239 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 36.380 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.221 / 3.164 / 8.109 / 27.819 ms (steady clock)
- DDS publish call mean / P95 / max: 0.058 / 0.295 / 1.452 ms (steady clock)

#### Command bridge — p5_retain_20261008_spin_spin_combined_3

- Wall duration: 79.518 s
- Received / published control frames: 3841 / 3841
- Published control FPS (ROS simulation time / active wall time): 60.000 / 58.791 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.156 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.009 / 15.908 / 22.490 / 86.311 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.084 / 0.081 / 0.107 / 0.973 ms (steady clock)
