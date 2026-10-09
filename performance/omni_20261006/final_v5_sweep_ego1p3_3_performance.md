
### Run final_v5_sweep_ego1p3_3

- Started: 2026-10-06 22:34:14 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v5_sweep_ego1p3_3

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 42.018 / 40.017 s
- Applied simulation control frames: 2403
- Runtime FPS (simulation time / wall time): 60.050 / 57.189 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.349 / 15.969 / 18.215 / 78.959 ms
- Distinct applied command values: 1905
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 20.884 / 16.532 / 33.118 / 895.525 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — final_v5_sweep_ego1p3_3

- Wall duration: 55.799 s
- Received / published control frames: 2401 / 2401
- Published control FPS (ROS simulation time / active wall time): 60.000 / 57.637 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.007 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.350 / 16.182 / 21.966 / 83.580 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.087 / 0.083 / 0.110 / 1.582 ms (steady clock)

#### Controller — final_v5_sweep_ego1p3_3

- Wall duration: 55.909 s
- Timer callbacks / command frames / guidance frames: 2402 / 2401 / 2220
- Command FPS (ROS simulation time / active wall time): 60.000 / 57.637 Hz
- Guidance FPS (active wall time): 57.094 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.671 / 0.220 / 2.455 / 11.313 ms
- Command interval mean / P50 / P95 / max: 17.350 / 16.182 / 21.962 / 83.901 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 41.336 / 33.333 / 83.333 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.735 / 2.470 / 6.904 / 15.892 ms (steady clock)
- DDS publish call mean / P95 / max: 0.059 / 0.132 / 2.143 ms (steady clock)
