
### Run p5_retain_20261008_spin_spin_v4_1

- Started: 2026-10-08 10:19:14 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_retain_20261008_spin_spin_v4_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 67.103 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 57.270 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.322 / 15.802 / 17.900 / 79.634 ms
- Distinct applied command values: 2972
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.083 / 16.032 / 32.928 / 4400.284 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_retain_20261008_spin_spin_v4_1

- Wall duration: 80.882 s
- Timer callbacks / command frames / guidance frames: 3810 / 3809 / 3268
- Command FPS (ROS simulation time / active wall time): 59.500 / 57.249 Hz
- Guidance FPS (active wall time): 52.948 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.807 / 16.667 / 16.667 / 150.000 ms
- Avoidance compute mean / P50 / P95 / max: 1.439 / 0.409 / 6.538 / 139.553 ms
- Command interval mean / P50 / P95 / max: 17.468 / 15.763 / 25.077 / 139.800 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 40.550 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.584 / 2.709 / 10.279 / 139.723 ms (steady clock)
- DDS publish call mean / P95 / max: 0.059 / 0.304 / 1.052 ms (steady clock)

#### Command bridge — p5_retain_20261008_spin_spin_v4_1

- Wall duration: 80.873 s
- Received / published control frames: 3809 / 3808
- Published control FPS (ROS simulation time / active wall time): 59.500 / 57.234 Hz
- Command stamp interval mean / P50 / P95 / max: 16.807 / 16.667 / 16.667 / 150.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.534 / 0.000 / 0.000 / 150.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.467 / 15.780 / 24.950 / 139.747 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.087 / 0.084 / 0.109 / 1.143 ms (steady clock)
