
### Run p5_enable_goal_20261008_dynamic_ego1p5_1

- Started: 2026-10-08 12:46:27 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `goal`

#### Isaac command application — p5_enable_goal_20261008_dynamic_ego1p5_1

- Result: TIMEOUT
- Wall duration / simulation duration: 226.024 / 242.000 s
- Applied simulation control frames: 14522
- Runtime FPS (simulation time / wall time): 60.008 / 64.250 Hz
- Isaac frame interval mean / P50 / P95 / max: 15.542 / 14.576 / 16.654 / 80.128 ms
- Distinct applied command values: 1
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_enable_goal_20261008_dynamic_ego1p5_1

- Wall duration: 240.462 s
- Timer callbacks / command frames / guidance frames: 14509 / 14508 / 14388
- Command FPS (ROS simulation time / active wall time): 59.950 / 64.291 Hz
- Guidance FPS (active wall time): 64.258 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.680 / 16.667 / 16.667 / 50.000 ms
- Avoidance compute mean / P50 / P95 / max: 3.181 / 0.873 / 16.342 / 36.590 ms
- Command interval mean / P50 / P95 / max: 15.554 / 14.056 / 32.615 / 96.079 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 38.963 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 5.311 / 3.672 / 19.136 / 50.701 ms (steady clock)
- DDS publish call mean / P95 / max: 0.053 / 0.256 / 2.002 ms (steady clock)

#### Command bridge — p5_enable_goal_20261008_dynamic_ego1p5_1

- Wall duration: 240.355 s
- Received / published control frames: 14508 / 14508
- Published control FPS (ROS simulation time / active wall time): 59.950 / 64.291 Hz
- Command stamp interval mean / P50 / P95 / max: 16.680 / 16.667 / 16.667 / 50.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 2.264 / 0.000 / 16.667 / 50.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 15.554 / 14.057 / 32.695 / 96.190 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.081 / 0.078 / 0.103 / 1.255 ms (steady clock)
