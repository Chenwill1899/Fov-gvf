
### Run p5_retain_goal_only_confirm_20261008_v4_ego1p5_1

- Started: 2026-10-08 11:14:07 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `goal`

#### Isaac command application — p5_retain_goal_only_confirm_20261008_v4_ego1p5_1

- Result: ARRIVED
- Wall duration / simulation duration: 74.998 / 72.800 s
- Applied simulation control frames: 4370
- Runtime FPS (simulation time / wall time): 60.027 / 58.268 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.087 / 15.896 / 17.927 / 79.405 ms
- Distinct applied command values: 3863
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 18.967 / 16.060 / 32.678 / 2000.793 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_retain_goal_only_confirm_20261008_v4_ego1p5_1

- Wall duration: 88.941 s
- Timer callbacks / command frames / guidance frames: 4356 / 4355 / 4165
- Command FPS (ROS simulation time / active wall time): 59.821 / 58.351 Hz
- Guidance FPS (active wall time): 58.440 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.716 / 16.667 / 16.667 / 100.000 ms
- Avoidance compute mean / P50 / P95 / max: 1.611 / 1.082 / 4.106 / 81.401 ms
- Command interval mean / P50 / P95 / max: 17.138 / 15.885 / 23.072 / 89.370 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 40.304 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.503 / 2.781 / 8.682 / 89.282 ms (steady clock)
- DDS publish call mean / P95 / max: 0.072 / 0.429 / 2.044 ms (steady clock)

#### Command bridge — p5_retain_goal_only_confirm_20261008_v4_ego1p5_1

- Wall duration: 88.832 s
- Received / published control frames: 4355 / 4355
- Published control FPS (ROS simulation time / active wall time): 59.821 / 58.351 Hz
- Command stamp interval mean / P50 / P95 / max: 16.716 / 16.667 / 16.667 / 100.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.149 / 0.000 / 0.000 / 100.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.138 / 15.876 / 23.048 / 89.504 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.087 / 0.083 / 0.109 / 1.777 ms (steady clock)
