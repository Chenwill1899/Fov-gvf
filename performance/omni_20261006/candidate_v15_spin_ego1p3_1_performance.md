
### Run candidate_v15_spin_ego1p3_1

- Started: 2026-10-06 23:53:59 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — candidate_v15_spin_ego1p3_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 66.684 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 57.630 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.213 / 16.010 / 18.089 / 78.872 ms
- Distinct applied command values: 3097
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 20.142 / 16.234 / 31.928 / 4197.187 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — candidate_v15_spin_ego1p3_1

- Wall duration: 80.862 s
- Timer callbacks / command frames / guidance frames: 3819 / 3818 / 3277
- Command FPS (ROS simulation time / active wall time): 59.641 / 57.751 Hz
- Guidance FPS (active wall time): 53.338 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.767 / 16.667 / 16.667 / 83.333 ms
- Avoidance compute mean / P50 / P95 / max: 2.405 / 1.166 / 7.528 / 58.892 ms
- Command interval mean / P50 / P95 / max: 17.316 / 16.047 / 23.108 / 84.629 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 45.916 / 33.333 / 83.333 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 4.165 / 3.603 / 10.435 / 75.421 ms (steady clock)
- DDS publish call mean / P95 / max: 0.063 / 0.389 / 2.115 ms (steady clock)

#### Command bridge — candidate_v15_spin_ego1p3_1

- Wall duration: 80.752 s
- Received / published control frames: 3818 / 3818
- Published control FPS (ROS simulation time / active wall time): 59.641 / 57.751 Hz
- Command stamp interval mean / P50 / P95 / max: 16.767 / 16.667 / 16.667 / 83.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.371 / 0.000 / 0.000 / 83.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.316 / 16.052 / 23.092 / 84.805 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.087 / 0.083 / 0.112 / 0.938 ms (steady clock)
