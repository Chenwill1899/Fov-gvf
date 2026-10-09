
### Run final_v22_sweep_ego1p3_3

- Started: 2026-10-07 02:01:31 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v22_sweep_ego1p3_3

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 41.870 / 40.017 s
- Applied simulation control frames: 2403
- Runtime FPS (simulation time / wall time): 60.050 / 57.392 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.291 / 15.808 / 18.047 / 78.668 ms
- Distinct applied command values: 1906
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 20.809 / 16.302 / 33.017 / 879.028 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — final_v22_sweep_ego1p3_3

- Wall duration: 55.826 s
- Received / published control frames: 2402 / 2402
- Published control FPS (ROS simulation time / active wall time): 60.000 / 57.363 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.014 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.433 / 16.116 / 22.680 / 373.940 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.086 / 0.084 / 0.112 / 0.469 ms (steady clock)

#### Controller — final_v22_sweep_ego1p3_3

- Wall duration: 55.933 s
- Timer callbacks / command frames / guidance frames: 2402 / 2402 / 2220
- Command FPS (ROS simulation time / active wall time): 60.000 / 57.362 Hz
- Guidance FPS (active wall time): 57.245 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.389 / 0.223 / 1.539 / 19.814 ms
- Command interval mean / P50 / P95 / max: 17.433 / 16.135 / 22.475 / 374.357 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 37.447 / 33.333 / 66.667 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.520 / 2.348 / 6.835 / 21.405 ms (steady clock)
- DDS publish call mean / P95 / max: 0.061 / 0.379 / 1.559 ms (steady clock)
