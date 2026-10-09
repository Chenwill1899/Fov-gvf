
### Run candidate_v21_spin_ego1p3_1

- Started: 2026-10-07 01:34:19 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — candidate_v21_spin_ego1p3_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 67.344 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 57.065 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.382 / 15.867 / 17.942 / 79.559 ms
- Distinct applied command values: 2804
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.371 / 16.353 / 33.082 / 4047.159 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — candidate_v21_spin_ego1p3_1

- Wall duration: 81.060 s
- Timer callbacks / command frames / guidance frames: 3826 / 3825 / 3284
- Command FPS (ROS simulation time / active wall time): 59.750 / 57.291 Hz
- Guidance FPS (active wall time): 53.161 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.736 / 16.667 / 16.667 / 83.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.965 / 0.413 / 3.823 / 74.174 ms
- Command interval mean / P50 / P95 / max: 17.455 / 15.927 / 25.132 / 93.831 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 44.189 / 33.333 / 83.333 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.062 / 2.400 / 9.095 / 85.760 ms (steady clock)
- DDS publish call mean / P95 / max: 0.064 / 0.407 / 1.284 ms (steady clock)

#### Command bridge — candidate_v21_spin_ego1p3_1

- Wall duration: 80.951 s
- Received / published control frames: 3825 / 3825
- Published control FPS (ROS simulation time / active wall time): 59.750 / 57.291 Hz
- Command stamp interval mean / P50 / P95 / max: 16.736 / 16.667 / 16.667 / 83.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.131 / 0.000 / 0.000 / 83.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.455 / 15.903 / 25.058 / 94.060 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.088 / 0.085 / 0.112 / 0.903 ms (steady clock)
