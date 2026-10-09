
### Run explore_v6_sweep_ego1p3_1

- Started: 2026-10-06 22:25:47 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — explore_v6_sweep_ego1p3_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 42.188 / 40.017 s
- Applied simulation control frames: 2403
- Runtime FPS (simulation time / wall time): 60.050 / 56.959 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.421 / 16.191 / 18.716 / 79.594 ms
- Distinct applied command values: 1897
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.005 / 16.690 / 33.449 / 964.158 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — explore_v6_sweep_ego1p3_1

- Wall duration: 56.070 s
- Timer callbacks / command frames / guidance frames: 2402 / 2401 / 2220
- Command FPS (ROS simulation time / active wall time): 60.000 / 57.507 Hz
- Guidance FPS (active wall time): 57.104 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.804 / 0.220 / 3.293 / 7.284 ms
- Command interval mean / P50 / P95 / max: 17.389 / 16.043 / 23.651 / 84.849 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 41.742 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.812 / 2.142 / 7.888 / 12.904 ms (steady clock)
- DDS publish call mean / P95 / max: 0.060 / 0.270 / 1.174 ms (steady clock)

#### Command bridge — explore_v6_sweep_ego1p3_1

- Wall duration: 55.959 s
- Received / published control frames: 2401 / 2401
- Published control FPS (ROS simulation time / active wall time): 60.000 / 57.507 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.389 / 16.035 / 23.465 / 85.383 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.088 / 0.085 / 0.111 / 0.653 ms (steady clock)
