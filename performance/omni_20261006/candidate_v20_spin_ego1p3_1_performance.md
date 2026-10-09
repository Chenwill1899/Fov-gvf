
### Run candidate_v20_spin_ego1p3_1

- Started: 2026-10-07 01:06:13 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — candidate_v20_spin_ego1p3_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 66.163 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 58.084 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.078 / 15.903 / 18.109 / 80.062 ms
- Distinct applied command values: 2883
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.448 / 16.284 / 32.967 / 4241.186 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — candidate_v20_spin_ego1p3_1

- Wall duration: 80.265 s
- Timer callbacks / command frames / guidance frames: 3763 / 3762 / 3221
- Command FPS (ROS simulation time / active wall time): 58.766 / 57.354 Hz
- Guidance FPS (active wall time): 52.834 Hz
- Command interval in ROS time mean / P50 / P95 / max: 17.017 / 16.667 / 16.667 / 116.667 ms
- Avoidance compute mean / P50 / P95 / max: 1.602 / 0.417 / 5.060 / 104.049 ms
- Command interval mean / P50 / P95 / max: 17.436 / 16.065 / 23.190 / 111.052 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 38.177 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.756 / 3.075 / 8.266 / 110.970 ms (steady clock)
- DDS publish call mean / P95 / max: 0.057 / 0.255 / 1.024 ms (steady clock)

#### Command bridge — candidate_v20_spin_ego1p3_1

- Wall duration: 80.158 s
- Received / published control frames: 3762 / 3761
- Published control FPS (ROS simulation time / active wall time): 58.766 / 57.339 Hz
- Command stamp interval mean / P50 / P95 / max: 17.017 / 16.667 / 16.667 / 116.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.567 / 0.000 / 0.000 / 116.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.436 / 16.037 / 23.207 / 110.992 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.086 / 0.083 / 0.111 / 0.939 ms (steady clock)
