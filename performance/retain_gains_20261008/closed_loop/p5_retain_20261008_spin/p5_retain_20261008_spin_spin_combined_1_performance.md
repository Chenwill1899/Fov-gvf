
### Run p5_retain_20261008_spin_spin_combined_1

- Started: 2026-10-08 10:15:30 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_retain_20261008_spin_spin_combined_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 66.205 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 58.047 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.088 / 15.872 / 17.830 / 79.922 ms
- Distinct applied command values: 3111
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 19.938 / 16.042 / 32.701 / 4158.988 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — p5_retain_20261008_spin_spin_combined_1

- Wall duration: 79.807 s
- Received / published control frames: 3826 / 3825
- Published control FPS (ROS simulation time / active wall time): 59.766 / 58.278 Hz
- Command stamp interval mean / P50 / P95 / max: 16.732 / 16.667 / 16.667 / 116.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.161 / 0.000 / 0.000 / 116.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.155 / 15.872 / 23.180 / 107.455 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.087 / 0.084 / 0.109 / 1.489 ms (steady clock)

#### Controller — p5_retain_20261008_spin_spin_combined_1

- Wall duration: 79.915 s
- Timer callbacks / command frames / guidance frames: 3827 / 3826 / 3285
- Command FPS (ROS simulation time / active wall time): 59.766 / 58.293 Hz
- Guidance FPS (active wall time): 53.732 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.732 / 16.667 / 16.667 / 116.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.849 / 0.405 / 3.376 / 105.648 ms
- Command interval mean / P50 / P95 / max: 17.155 / 15.907 / 23.209 / 107.444 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 39.462 / 33.333 / 83.333 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.872 / 2.684 / 7.401 / 107.390 ms (steady clock)
- DDS publish call mean / P95 / max: 0.061 / 0.352 / 1.529 ms (steady clock)
