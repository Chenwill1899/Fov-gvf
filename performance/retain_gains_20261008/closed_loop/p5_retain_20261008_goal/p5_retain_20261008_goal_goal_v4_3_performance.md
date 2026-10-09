
### Run p5_retain_20261008_goal_v4_ego1p5_3

- Started: 2026-10-08 10:39:08 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `goal`

#### Isaac command application — p5_retain_20261008_goal_v4_ego1p5_3

- Result: ARRIVED
- Wall duration / simulation duration: 72.705 / 71.233 s
- Applied simulation control frames: 4276
- Runtime FPS (simulation time / wall time): 60.028 / 58.813 Hz
- Isaac frame interval mean / P50 / P95 / max: 16.927 / 15.722 / 17.596 / 79.442 ms
- Distinct applied command values: 3299
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.577 / 15.892 / 32.404 / 3096.779 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_retain_20261008_goal_v4_ego1p5_3

- Wall duration: 86.624 s
- Timer callbacks / command frames / guidance frames: 4262 / 4261 / 4071
- Command FPS (ROS simulation time / active wall time): 59.817 / 58.897 Hz
- Guidance FPS (active wall time): 58.779 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.718 / 16.667 / 16.667 / 50.000 ms
- Avoidance compute mean / P50 / P95 / max: 1.375 / 0.811 / 3.516 / 42.292 ms
- Command interval mean / P50 / P95 / max: 16.979 / 15.925 / 21.941 / 82.806 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 41.198 / 33.333 / 83.333 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.380 / 2.873 / 8.063 / 43.835 ms (steady clock)
- DDS publish call mean / P95 / max: 0.061 / 0.375 / 1.614 ms (steady clock)

#### Command bridge — p5_retain_20261008_goal_v4_ego1p5_3

- Wall duration: 86.517 s
- Received / published control frames: 4261 / 4261
- Published control FPS (ROS simulation time / active wall time): 59.817 / 58.897 Hz
- Command stamp interval mean / P50 / P95 / max: 16.718 / 16.667 / 16.667 / 50.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.192 / 0.000 / 0.000 / 50.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 16.979 / 15.918 / 21.948 / 82.920 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.088 / 0.084 / 0.114 / 0.935 ms (steady clock)
