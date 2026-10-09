
### Run p5_retain_goal_only_confirm_20261008_baseline_ego1p5_3

- Started: 2026-10-08 11:20:01 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `goal`

#### Isaac command application — p5_retain_goal_only_confirm_20261008_baseline_ego1p5_3

- Result: ARRIVED
- Wall duration / simulation duration: 76.257 / 73.867 s
- Applied simulation control frames: 4434
- Runtime FPS (simulation time / wall time): 60.027 / 58.145 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.126 / 15.822 / 17.990 / 79.584 ms
- Distinct applied command values: 3810
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 19.615 / 16.047 / 32.835 / 1807.646 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — p5_retain_goal_only_confirm_20261008_baseline_ego1p5_3

- Wall duration: 90.136 s
- Received / published control frames: 4358 / 4357
- Published control FPS (ROS simulation time / active wall time): 58.998 / 57.409 Hz
- Command stamp interval mean / P50 / P95 / max: 16.950 / 16.667 / 16.667 / 733.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.478 / 0.000 / 0.000 / 733.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.415 / 15.820 / 23.492 / 755.946 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.090 / 0.085 / 0.114 / 1.146 ms (steady clock)

#### Controller — p5_retain_goal_only_confirm_20261008_baseline_ego1p5_3

- Wall duration: 90.243 s
- Timer callbacks / command frames / guidance frames: 4359 / 4358 / 4168
- Command FPS (ROS simulation time / active wall time): 58.998 / 57.422 Hz
- Guidance FPS (active wall time): 57.155 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.950 / 16.667 / 16.667 / 733.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.961 / 1.185 / 4.515 / 748.203 ms
- Command interval mean / P50 / P95 / max: 17.415 / 15.797 / 23.533 / 755.931 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 41.751 / 33.333 / 83.333 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.775 / 2.645 / 8.931 / 755.799 ms (steady clock)
- DDS publish call mean / P95 / max: 0.067 / 0.421 / 2.608 ms (steady clock)
