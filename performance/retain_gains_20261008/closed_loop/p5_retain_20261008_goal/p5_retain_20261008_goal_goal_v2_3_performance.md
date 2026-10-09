
### Run p5_retain_20261008_goal_v2_ego1p5_3

- Started: 2026-10-08 10:46:29 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `goal`

#### Isaac command application — p5_retain_20261008_goal_v2_ego1p5_3

- Result: ARRIVED
- Wall duration / simulation duration: 75.133 / 71.167 s
- Applied simulation control frames: 4272
- Runtime FPS (simulation time / wall time): 60.028 / 56.859 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.513 / 15.957 / 18.029 / 84.326 ms
- Distinct applied command values: 3247
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.708 / 16.098 / 33.424 / 2967.745 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — p5_retain_20261008_goal_v2_ego1p5_3

- Wall duration: 88.945 s
- Received / published control frames: 4266 / 4266
- Published control FPS (ROS simulation time / active wall time): 59.944 / 57.048 Hz
- Command stamp interval mean / P50 / P95 / max: 16.682 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.133 / 0.000 / 0.000 / 33.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.529 / 16.131 / 22.150 / 91.032 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.089 / 0.086 / 0.116 / 0.969 ms (steady clock)

#### Controller — p5_retain_20261008_goal_v2_ego1p5_3

- Wall duration: 89.053 s
- Timer callbacks / command frames / guidance frames: 4267 / 4266 / 4076
- Command FPS (ROS simulation time / active wall time): 59.944 / 57.048 Hz
- Guidance FPS (active wall time): 56.866 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.682 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.318 / 0.842 / 3.560 / 21.511 ms
- Command interval mean / P50 / P95 / max: 17.529 / 16.141 / 22.158 / 91.101 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 40.244 / 33.333 / 83.333 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.316 / 2.986 / 7.926 / 29.103 ms (steady clock)
- DDS publish call mean / P95 / max: 0.066 / 0.421 / 1.904 ms (steady clock)
