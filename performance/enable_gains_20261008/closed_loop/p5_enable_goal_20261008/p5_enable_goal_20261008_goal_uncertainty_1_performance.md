
### Run p5_enable_goal_20261008_uncertainty_ego1p5_1

- Started: 2026-10-08 12:50:30 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `goal`

#### Isaac command application — p5_enable_goal_20261008_uncertainty_ego1p5_1

- Result: ARRIVED
- Wall duration / simulation duration: 80.912 / 73.000 s
- Applied simulation control frames: 4382
- Runtime FPS (simulation time / wall time): 60.027 / 54.157 Hz
- Isaac frame interval mean / P50 / P95 / max: 18.383 / 16.923 / 19.779 / 82.718 ms
- Distinct applied command values: 3977
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 19.939 / 17.037 / 33.994 / 1914.581 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_enable_goal_20261008_uncertainty_ego1p5_1

- Wall duration: 95.165 s
- Timer callbacks / command frames / guidance frames: 4377 / 4376 / 4185
- Command FPS (ROS simulation time / active wall time): 59.945 / 54.348 Hz
- Guidance FPS (active wall time): 54.082 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.682 / 16.667 / 16.667 / 50.000 ms
- Avoidance compute mean / P50 / P95 / max: 1.759 / 1.221 / 4.697 / 78.859 ms
- Command interval mean / P50 / P95 / max: 18.400 / 16.768 / 23.744 / 86.664 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 40.442 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.575 / 3.046 / 8.168 / 80.880 ms (steady clock)
- DDS publish call mean / P95 / max: 0.063 / 0.353 / 2.363 ms (steady clock)

#### Command bridge — p5_enable_goal_20261008_uncertainty_ego1p5_1

- Wall duration: 95.059 s
- Received / published control frames: 4376 / 4376
- Published control FPS (ROS simulation time / active wall time): 59.945 / 54.348 Hz
- Command stamp interval mean / P50 / P95 / max: 16.682 / 16.667 / 16.667 / 50.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.114 / 0.000 / 0.000 / 66.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.400 / 16.741 / 23.759 / 86.635 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.096 / 0.092 / 0.126 / 1.362 ms (steady clock)
