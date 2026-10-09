
### Run p5_goal_polish_explore_20261007_after_ego1p5_1

- Started: 2026-10-07 23:01:07 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `goal`

#### Isaac command application — p5_goal_polish_explore_20261007_after_ego1p5_1

- Result: ARRIVED
- Wall duration / simulation duration: 84.964 / 81.967 s
- Applied simulation control frames: 4920
- Runtime FPS (simulation time / wall time): 60.024 / 57.907 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.203 / 15.933 / 17.958 / 78.520 ms
- Distinct applied command values: 3743
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.288 / 16.276 / 48.159 / 1836.599 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_goal_polish_explore_20261007_after_ego1p5_1

- Wall duration: 98.838 s
- Timer callbacks / command frames / guidance frames: 4412 / 4411 / 4222
- Command FPS (ROS simulation time / active wall time): 53.813 / 52.134 Hz
- Guidance FPS (active wall time): 51.752 Hz
- Command interval in ROS time mean / P50 / P95 / max: 18.583 / 16.667 / 33.333 / 116.667 ms
- Avoidance compute mean / P50 / P95 / max: 5.911 / 2.157 / 25.830 / 169.805 ms
- Command interval mean / P50 / P95 / max: 19.181 / 16.476 / 39.241 / 172.282 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 36.709 / 33.333 / 66.667 / 83.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 8.235 / 4.846 / 30.724 / 172.215 ms (steady clock)
- DDS publish call mean / P95 / max: 0.065 / 0.403 / 2.624 ms (steady clock)

#### Command bridge — p5_goal_polish_explore_20261007_after_ego1p5_1

- Wall duration: 98.732 s
- Received / published control frames: 4411 / 4408
- Published control FPS (ROS simulation time / active wall time): 53.813 / 52.099 Hz
- Command stamp interval mean / P50 / P95 / max: 18.583 / 16.667 / 33.333 / 116.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 4.047 / 0.000 / 33.333 / 116.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 19.181 / 16.464 / 39.163 / 173.111 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.090 / 0.086 / 0.118 / 1.206 ms (steady clock)
