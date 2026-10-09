
### Run p5_goal_polish_explore_20261007_after_ego1p5_3

- Started: 2026-10-07 23:10:49 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `goal`

#### Isaac command application — p5_goal_polish_explore_20261007_after_ego1p5_3

- Result: TIMEOUT
- Wall duration / simulation duration: 227.617 / 242.000 s
- Applied simulation control frames: 14522
- Runtime FPS (simulation time / wall time): 60.008 / 63.800 Hz
- Isaac frame interval mean / P50 / P95 / max: 15.651 / 14.740 / 17.280 / 80.915 ms
- Distinct applied command values: 2734
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 26.325 / 16.445 / 64.168 / 3418.633 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_goal_polish_explore_20261007_after_ego1p5_3

- Wall duration: 241.542 s
- Timer callbacks / command frames / guidance frames: 4686 / 4685 / 4565
- Command FPS (ROS simulation time / active wall time): 19.357 / 20.598 Hz
- Guidance FPS (active wall time): 20.237 Hz
- Command interval in ROS time mean / P50 / P95 / max: 51.662 / 16.667 / 150.000 / 350.000 ms
- Avoidance compute mean / P50 / P95 / max: 34.148 / 6.393 / 107.609 / 318.227 ms
- Command interval mean / P50 / P95 / max: 48.549 / 18.841 / 130.726 / 330.160 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 35.900 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 41.078 / 8.824 / 130.606 / 330.023 ms (steady clock)
- DDS publish call mean / P95 / max: 0.059 / 0.117 / 1.119 ms (steady clock)

#### Command bridge — p5_goal_polish_explore_20261007_after_ego1p5_3

- Wall duration: 241.433 s
- Received / published control frames: 4685 / 3571
- Published control FPS (ROS simulation time / active wall time): 19.357 / 15.699 Hz
- Command stamp interval mean / P50 / P95 / max: 51.662 / 16.667 / 150.000 / 350.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 41.942 / 0.000 / 150.000 / 350.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 48.549 / 18.854 / 130.762 / 330.224 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.091 / 0.088 / 0.119 / 1.629 ms (steady clock)
