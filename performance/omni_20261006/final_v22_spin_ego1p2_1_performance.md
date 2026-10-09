
### Run final_v22_spin_ego1p2_1

- Started: 2026-10-07 02:02:28 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v22_spin_ego1p2_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 67.070 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 57.299 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.314 / 15.978 / 18.256 / 80.062 ms
- Distinct applied command values: 2838
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.071 / 16.421 / 33.194 / 4387.427 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — final_v22_spin_ego1p2_1

- Wall duration: 80.903 s
- Timer callbacks / command frames / guidance frames: 3839 / 3838 / 3297
- Command FPS (ROS simulation time / active wall time): 59.953 / 57.713 Hz
- Guidance FPS (active wall time): 53.433 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.680 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.680 / 0.532 / 1.438 / 7.211 ms
- Command interval mean / P50 / P95 / max: 17.327 / 16.069 / 23.961 / 95.532 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 71.418 / 66.667 / 116.667 / 133.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.792 / 1.213 / 7.741 / 18.380 ms (steady clock)
- DDS publish call mean / P95 / max: 0.056 / 0.137 / 1.832 ms (steady clock)

#### Command bridge — final_v22_spin_ego1p2_1

- Wall duration: 80.793 s
- Received / published control frames: 3838 / 3838
- Published control FPS (ROS simulation time / active wall time): 59.953 / 57.713 Hz
- Command stamp interval mean / P50 / P95 / max: 16.680 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.026 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.327 / 16.079 / 23.970 / 95.577 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.088 / 0.085 / 0.113 / 1.715 ms (steady clock)
