
### Run final_v22_sweep_ego1p3_2

- Started: 2026-10-07 01:58:42 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v22_sweep_ego1p3_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 41.042 / 40.017 s
- Applied simulation control frames: 2403
- Runtime FPS (simulation time / wall time): 60.050 / 58.550 Hz
- Isaac frame interval mean / P50 / P95 / max: 16.945 / 15.832 / 17.732 / 78.523 ms
- Distinct applied command values: 1889
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 20.488 / 16.391 / 32.547 / 942.220 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — final_v22_sweep_ego1p3_2

- Wall duration: 55.225 s
- Received / published control frames: 2401 / 2401
- Published control FPS (ROS simulation time / active wall time): 60.000 / 59.019 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 16.944 / 15.884 / 22.765 / 84.578 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.086 / 0.084 / 0.106 / 0.633 ms (steady clock)

#### Controller — final_v22_sweep_ego1p3_2

- Wall duration: 55.332 s
- Timer callbacks / command frames / guidance frames: 2402 / 2401 / 2220
- Command FPS (ROS simulation time / active wall time): 60.000 / 59.019 Hz
- Guidance FPS (active wall time): 58.793 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.367 / 0.224 / 1.126 / 8.143 ms
- Command interval mean / P50 / P95 / max: 16.944 / 15.862 / 22.841 / 84.530 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 39.384 / 33.333 / 83.333 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.488 / 2.101 / 6.808 / 16.543 ms (steady clock)
- DDS publish call mean / P95 / max: 0.061 / 0.283 / 2.493 ms (steady clock)
