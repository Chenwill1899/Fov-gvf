
### Run final_v22_sweep_ego1p2_1

- Started: 2026-10-07 01:56:48 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v22_sweep_ego1p2_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 41.688 / 40.017 s
- Applied simulation control frames: 2403
- Runtime FPS (simulation time / wall time): 60.050 / 57.642 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.187 / 15.733 / 17.703 / 78.802 ms
- Distinct applied command values: 1705
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 23.122 / 16.337 / 35.079 / 900.720 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — final_v22_sweep_ego1p2_1

- Wall duration: 55.532 s
- Received / published control frames: 2314 / 2312
- Published control FPS (ROS simulation time / active wall time): 57.825 / 56.047 Hz
- Command stamp interval mean / P50 / P95 / max: 17.294 / 16.667 / 16.667 / 250.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.944 / 0.000 / 0.000 / 250.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.827 / 15.836 / 23.161 / 235.990 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.085 / 0.082 / 0.110 / 1.048 ms (steady clock)

#### Controller — final_v22_sweep_ego1p2_1

- Wall duration: 55.639 s
- Timer callbacks / command frames / guidance frames: 2315 / 2314 / 2133
- Command FPS (ROS simulation time / active wall time): 57.825 / 56.096 Hz
- Guidance FPS (active wall time): 55.352 Hz
- Command interval in ROS time mean / P50 / P95 / max: 17.294 / 16.667 / 16.667 / 250.000 ms
- Avoidance compute mean / P50 / P95 / max: 1.467 / 0.370 / 1.500 / 222.869 ms
- Command interval mean / P50 / P95 / max: 17.827 / 15.885 / 23.127 / 235.973 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 73.074 / 66.667 / 116.667 / 133.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.443 / 1.753 / 7.156 / 235.860 ms (steady clock)
- DDS publish call mean / P95 / max: 0.054 / 0.111 / 1.573 ms (steady clock)
