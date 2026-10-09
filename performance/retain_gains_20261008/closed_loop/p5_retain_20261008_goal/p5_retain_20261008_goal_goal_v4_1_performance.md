
### Run p5_retain_20261008_goal_v4_ego1p5_1

- Started: 2026-10-08 10:20:43 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `goal`

#### Isaac command application — p5_retain_20261008_goal_v4_ego1p5_1

- Result: ARRIVED
- Wall duration / simulation duration: 74.471 / 71.217 s
- Applied simulation control frames: 4275
- Runtime FPS (simulation time / wall time): 60.028 / 57.405 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.343 / 15.872 / 17.901 / 79.368 ms
- Distinct applied command values: 3163
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 23.058 / 16.104 / 33.213 / 4416.935 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — p5_retain_20261008_goal_v4_ego1p5_1

- Wall duration: 88.313 s
- Received / published control frames: 4264 / 4264
- Published control FPS (ROS simulation time / active wall time): 59.874 / 57.541 Hz
- Command stamp interval mean / P50 / P95 / max: 16.702 / 16.667 / 16.667 / 50.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.176 / 0.000 / 0.000 / 50.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.379 / 15.932 / 23.351 / 85.492 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.088 / 0.085 / 0.113 / 1.210 ms (steady clock)

#### Controller — p5_retain_20261008_goal_v4_ego1p5_1

- Wall duration: 88.422 s
- Timer callbacks / command frames / guidance frames: 4265 / 4264 / 4074
- Command FPS (ROS simulation time / active wall time): 59.874 / 57.541 Hz
- Guidance FPS (active wall time): 57.247 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.702 / 16.667 / 16.667 / 50.000 ms
- Avoidance compute mean / P50 / P95 / max: 1.391 / 0.811 / 3.873 / 52.692 ms
- Command interval mean / P50 / P95 / max: 17.379 / 15.939 / 23.359 / 85.696 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 37.658 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.412 / 2.876 / 8.298 / 54.837 ms (steady clock)
- DDS publish call mean / P95 / max: 0.065 / 0.414 / 1.451 ms (steady clock)
