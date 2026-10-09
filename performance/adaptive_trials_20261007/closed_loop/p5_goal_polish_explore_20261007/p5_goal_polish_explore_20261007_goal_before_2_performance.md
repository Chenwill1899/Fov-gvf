
### Run p5_goal_polish_explore_20261007_before_ego1p5_2

- Started: 2026-10-07 23:06:10 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `goal`

#### Isaac command application — p5_goal_polish_explore_20261007_before_ego1p5_2

- Result: ARRIVED
- Wall duration / simulation duration: 74.435 / 71.167 s
- Applied simulation control frames: 4272
- Runtime FPS (simulation time / wall time): 60.028 / 57.392 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.346 / 15.947 / 18.062 / 78.615 ms
- Distinct applied command values: 3203
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.776 / 16.153 / 32.682 / 4629.625 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — p5_goal_polish_explore_20261007_before_ego1p5_2

- Wall duration: 88.322 s
- Received / published control frames: 4266 / 4266
- Published control FPS (ROS simulation time / active wall time): 59.944 / 57.598 Hz
- Command stamp interval mean / P50 / P95 / max: 16.682 / 16.667 / 16.667 / 50.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.094 / 0.000 / 0.000 / 50.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.362 / 16.274 / 22.398 / 84.345 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.089 / 0.085 / 0.111 / 1.158 ms (steady clock)

#### Controller — p5_goal_polish_explore_20261007_before_ego1p5_2

- Wall duration: 88.431 s
- Timer callbacks / command frames / guidance frames: 4267 / 4266 / 4076
- Command FPS (ROS simulation time / active wall time): 59.944 / 57.597 Hz
- Guidance FPS (active wall time): 57.301 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.682 / 16.667 / 16.667 / 50.000 ms
- Avoidance compute mean / P50 / P95 / max: 1.361 / 0.887 / 3.765 / 52.485 ms
- Command interval mean / P50 / P95 / max: 17.362 / 16.302 / 22.380 / 84.458 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 35.999 / 33.333 / 50.000 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.434 / 3.214 / 8.037 / 52.630 ms (steady clock)
- DDS publish call mean / P95 / max: 0.066 / 0.421 / 1.316 ms (steady clock)
