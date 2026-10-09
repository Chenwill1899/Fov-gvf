
### Run p5_retain_20261008_goal_v4_ego1p5_2

- Started: 2026-10-08 10:28:05 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `goal`

#### Isaac command application — p5_retain_20261008_goal_v4_ego1p5_2

- Result: ARRIVED
- Wall duration / simulation duration: 73.450 / 71.267 s
- Applied simulation control frames: 4278
- Runtime FPS (simulation time / wall time): 60.028 / 58.244 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.092 / 15.748 / 17.734 / 78.580 ms
- Distinct applied command values: 3206
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.453 / 15.911 / 32.476 / 4407.678 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — p5_retain_20261008_goal_v4_ego1p5_2

- Wall duration: 87.318 s
- Received / published control frames: 4275 / 4275
- Published control FPS (ROS simulation time / active wall time): 59.986 / 58.492 Hz
- Command stamp interval mean / P50 / P95 / max: 16.671 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.082 / 0.000 / 0.000 / 33.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.096 / 15.856 / 22.019 / 82.144 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.087 / 0.084 / 0.112 / 1.089 ms (steady clock)

#### Controller — p5_retain_20261008_goal_v4_ego1p5_2

- Wall duration: 87.425 s
- Timer callbacks / command frames / guidance frames: 4276 / 4275 / 4085
- Command FPS (ROS simulation time / active wall time): 59.986 / 58.492 Hz
- Guidance FPS (active wall time): 58.422 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.671 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.216 / 0.803 / 3.405 / 21.398 ms
- Command interval mean / P50 / P95 / max: 17.096 / 15.879 / 21.986 / 82.026 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 38.266 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.270 / 2.943 / 7.484 / 25.323 ms (steady clock)
- DDS publish call mean / P95 / max: 0.063 / 0.393 / 1.487 ms (steady clock)
