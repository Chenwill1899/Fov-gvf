
### Run p5_retain_20261008_spin_spin_v2_1

- Started: 2026-10-08 10:11:51 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_retain_20261008_spin_spin_v2_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 65.507 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 58.665 Hz
- Isaac frame interval mean / P50 / P95 / max: 16.905 / 15.830 / 17.738 / 78.140 ms
- Distinct applied command values: 3177
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 19.322 / 15.992 / 30.787 / 4090.985 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_retain_20261008_spin_spin_v2_1

- Wall duration: 79.232 s
- Timer callbacks / command frames / guidance frames: 3827 / 3826 / 3285
- Command FPS (ROS simulation time / active wall time): 59.766 / 58.922 Hz
- Guidance FPS (active wall time): 54.360 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.732 / 16.667 / 16.667 / 166.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.841 / 0.407 / 3.215 / 145.203 ms
- Command interval mean / P50 / P95 / max: 16.971 / 15.927 / 22.270 / 150.289 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 40.832 / 33.333 / 83.333 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.931 / 2.785 / 6.937 / 150.214 ms (steady clock)
- DDS publish call mean / P95 / max: 0.058 / 0.177 / 1.364 ms (steady clock)

#### Command bridge — p5_retain_20261008_spin_spin_v2_1

- Wall duration: 79.120 s
- Received / published control frames: 3826 / 3825
- Published control FPS (ROS simulation time / active wall time): 59.766 / 58.907 Hz
- Command stamp interval mean / P50 / P95 / max: 16.732 / 16.667 / 16.667 / 166.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.126 / 0.000 / 0.000 / 166.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 16.971 / 15.933 / 22.239 / 150.249 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.089 / 0.086 / 0.112 / 1.762 ms (steady clock)
