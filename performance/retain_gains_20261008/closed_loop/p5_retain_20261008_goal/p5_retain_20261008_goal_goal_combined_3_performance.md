
### Run p5_retain_20261008_goal_combined_ego1p5_3

- Started: 2026-10-08 10:42:46 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `goal`

#### Isaac command application — p5_retain_20261008_goal_combined_ego1p5_3

- Result: ARRIVED
- Wall duration / simulation duration: 76.987 / 74.583 s
- Applied simulation control frames: 4477
- Runtime FPS (simulation time / wall time): 60.027 / 58.152 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.125 / 15.763 / 17.769 / 77.763 ms
- Distinct applied command values: 3981
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 18.943 / 15.938 / 32.327 / 1790.892 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — p5_retain_20261008_goal_combined_ego1p5_3

- Wall duration: 90.835 s
- Received / published control frames: 4455 / 4455
- Published control FPS (ROS simulation time / active wall time): 59.732 / 58.133 Hz
- Command stamp interval mean / P50 / P95 / max: 16.742 / 16.667 / 16.667 / 50.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.382 / 0.000 / 0.000 / 50.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.202 / 15.978 / 22.480 / 82.870 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.088 / 0.084 / 0.113 / 1.100 ms (steady clock)

#### Controller — p5_retain_20261008_goal_combined_ego1p5_3

- Wall duration: 90.945 s
- Timer callbacks / command frames / guidance frames: 4456 / 4455 / 4265
- Command FPS (ROS simulation time / active wall time): 59.732 / 58.132 Hz
- Guidance FPS (active wall time): 57.932 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.742 / 16.667 / 16.667 / 50.000 ms
- Avoidance compute mean / P50 / P95 / max: 1.807 / 1.117 / 4.925 / 38.920 ms
- Command interval mean / P50 / P95 / max: 17.202 / 15.944 / 22.497 / 82.770 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 38.570 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.728 / 2.970 / 9.247 / 43.517 ms (steady clock)
- DDS publish call mean / P95 / max: 0.066 / 0.412 / 1.571 ms (steady clock)
