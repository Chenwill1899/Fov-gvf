
### Run p5_goal_polish_explore_20261007_after_ego1p5_2

- Started: 2026-10-07 23:03:38 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `goal`

#### Isaac command application — p5_goal_polish_explore_20261007_after_ego1p5_2

- Result: ARRIVED
- Wall duration / simulation duration: 85.355 / 82.183 s
- Applied simulation control frames: 4933
- Runtime FPS (simulation time / wall time): 60.024 / 57.794 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.238 / 15.906 / 18.000 / 78.681 ms
- Distinct applied command values: 3773
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.251 / 16.283 / 48.109 / 1746.365 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_goal_polish_explore_20261007_after_ego1p5_2

- Wall duration: 99.305 s
- Timer callbacks / command frames / guidance frames: 4384 / 4383 / 4193
- Command FPS (ROS simulation time / active wall time): 53.331 / 51.564 Hz
- Guidance FPS (active wall time): 51.004 Hz
- Command interval in ROS time mean / P50 / P95 / max: 18.751 / 16.667 / 33.333 / 816.667 ms
- Avoidance compute mean / P50 / P95 / max: 6.446 / 2.407 / 26.308 / 745.310 ms
- Command interval mean / P50 / P95 / max: 19.393 / 16.573 / 40.194 / 751.335 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 37.487 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 8.780 / 4.992 / 31.128 / 751.248 ms (steady clock)
- DDS publish call mean / P95 / max: 0.067 / 0.417 / 1.330 ms (steady clock)

#### Command bridge — p5_goal_polish_explore_20261007_after_ego1p5_2

- Wall duration: 99.295 s
- Received / published control frames: 4383 / 4378
- Published control FPS (ROS simulation time / active wall time): 53.331 / 51.505 Hz
- Command stamp interval mean / P50 / P95 / max: 18.751 / 16.667 / 33.333 / 816.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 4.651 / 0.000 / 33.333 / 816.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 19.393 / 16.542 / 40.280 / 751.321 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.087 / 0.083 / 0.115 / 1.130 ms (steady clock)
