
### Run p5_retain_20261008_goal_v2_ego1p5_1

- Started: 2026-10-08 10:13:19 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `goal`

#### Isaac command application — p5_retain_20261008_goal_v2_ego1p5_1

- Result: ARRIVED
- Wall duration / simulation duration: 77.050 / 74.017 s
- Applied simulation control frames: 4443
- Runtime FPS (simulation time / wall time): 60.027 / 57.664 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.268 / 15.965 / 17.946 / 79.839 ms
- Distinct applied command values: 3936
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 19.219 / 16.118 / 32.840 / 1873.276 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_retain_20261008_goal_v2_ego1p5_1

- Wall duration: 91.018 s
- Timer callbacks / command frames / guidance frames: 4389 / 4388 / 4198
- Command FPS (ROS simulation time / active wall time): 59.284 / 57.222 Hz
- Guidance FPS (active wall time): 56.907 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.868 / 16.667 / 16.667 / 83.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.856 / 1.113 / 4.029 / 81.300 ms
- Command interval mean / P50 / P95 / max: 17.476 / 15.928 / 22.546 / 82.675 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 40.392 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.754 / 2.978 / 8.137 / 81.475 ms (steady clock)
- DDS publish call mean / P95 / max: 0.069 / 0.414 / 2.164 ms (steady clock)

#### Command bridge — p5_retain_20261008_goal_v2_ego1p5_1

- Wall duration: 90.908 s
- Received / published control frames: 4388 / 4388
- Published control FPS (ROS simulation time / active wall time): 59.284 / 57.221 Hz
- Command stamp interval mean / P50 / P95 / max: 16.868 / 16.667 / 16.667 / 83.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.403 / 0.000 / 0.000 / 83.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.476 / 15.920 / 22.560 / 82.672 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.090 / 0.085 / 0.116 / 1.122 ms (steady clock)
