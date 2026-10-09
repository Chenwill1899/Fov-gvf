
### Run p5_enable_goal_20261008_shared_ego1p5_1

- Started: 2026-10-08 12:43:23 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `goal`

#### Isaac command application — p5_enable_goal_20261008_shared_ego1p5_1

- Result: ARRIVED
- Wall duration / simulation duration: 75.579 / 71.567 s
- Applied simulation control frames: 4296
- Runtime FPS (simulation time / wall time): 60.028 / 56.841 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.516 / 16.025 / 18.177 / 80.697 ms
- Distinct applied command values: 2957
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 25.080 / 16.338 / 34.443 / 4596.553 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — p5_enable_goal_20261008_shared_ego1p5_1

- Wall duration: 89.503 s
- Received / published control frames: 4281 / 4281
- Published control FPS (ROS simulation time / active wall time): 59.804 / 56.639 Hz
- Command stamp interval mean / P50 / P95 / max: 16.721 / 16.667 / 16.667 / 83.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.300 / 0.000 / 0.000 / 83.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.656 / 15.637 / 27.843 / 370.710 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.087 / 0.084 / 0.114 / 0.783 ms (steady clock)

#### Controller — p5_enable_goal_20261008_shared_ego1p5_1

- Wall duration: 89.944 s
- Timer callbacks / command frames / guidance frames: 4281 / 4281 / 4091
- Command FPS (ROS simulation time / active wall time): 59.804 / 56.639 Hz
- Guidance FPS (active wall time): 56.594 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.721 / 16.667 / 16.667 / 83.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.489 / 0.859 / 3.797 / 72.576 ms
- Command interval mean / P50 / P95 / max: 17.656 / 15.603 / 27.961 / 370.324 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 45.657 / 50.000 / 83.333 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 4.408 / 3.484 / 12.650 / 83.415 ms (steady clock)
- DDS publish call mean / P95 / max: 0.064 / 0.423 / 1.879 ms (steady clock)
