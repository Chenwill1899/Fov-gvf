
### Run p5_goal_polish_explore_20261007_before_ego1p5_1

- Started: 2026-10-07 22:58:50 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `goal`

#### Isaac command application — p5_goal_polish_explore_20261007_before_ego1p5_1

- Result: ARRIVED
- Wall duration / simulation duration: 72.913 / 71.233 s
- Applied simulation control frames: 4276
- Runtime FPS (simulation time / wall time): 60.028 / 58.645 Hz
- Isaac frame interval mean / P50 / P95 / max: 16.977 / 15.689 / 17.765 / 79.522 ms
- Distinct applied command values: 3152
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.631 / 15.882 / 32.681 / 4683.824 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_goal_polish_explore_20261007_before_ego1p5_1

- Wall duration: 86.841 s
- Timer callbacks / command frames / guidance frames: 4274 / 4273 / 4083
- Command FPS (ROS simulation time / active wall time): 59.986 / 58.891 Hz
- Guidance FPS (active wall time): 58.719 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.671 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.225 / 0.817 / 3.453 / 14.955 ms
- Command interval mean / P50 / P95 / max: 16.981 / 15.861 / 22.866 / 94.697 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 45.432 / 33.333 / 83.333 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.254 / 2.706 / 8.647 / 24.448 ms (steady clock)
- DDS publish call mean / P95 / max: 0.067 / 0.418 / 1.422 ms (steady clock)

#### Command bridge — p5_goal_polish_explore_20261007_before_ego1p5_1

- Wall duration: 86.735 s
- Received / published control frames: 4273 / 4273
- Published control FPS (ROS simulation time / active wall time): 59.986 / 58.891 Hz
- Command stamp interval mean / P50 / P95 / max: 16.671 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.059 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 16.980 / 15.861 / 22.813 / 94.756 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.088 / 0.085 / 0.114 / 1.574 ms (steady clock)
