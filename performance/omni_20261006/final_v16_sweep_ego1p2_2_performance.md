
### Run final_v16_sweep_ego1p2_2

- Started: 2026-10-07 00:06:44 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v16_sweep_ego1p2_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 40.930 / 40.017 s
- Applied simulation control frames: 2403
- Runtime FPS (simulation time / wall time): 60.050 / 58.710 Hz
- Isaac frame interval mean / P50 / P95 / max: 16.895 / 15.885 / 18.081 / 79.363 ms
- Distinct applied command values: 1723
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.498 / 16.437 / 33.330 / 944.418 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — final_v16_sweep_ego1p2_2

- Wall duration: 54.940 s
- Timer callbacks / command frames / guidance frames: 2307 / 2306 / 2125
- Command FPS (ROS simulation time / active wall time): 57.625 / 56.852 Hz
- Guidance FPS (active wall time): 56.191 Hz
- Command interval in ROS time mean / P50 / P95 / max: 17.354 / 16.667 / 16.667 / 250.000 ms
- Avoidance compute mean / P50 / P95 / max: 1.468 / 0.383 / 1.533 / 224.993 ms
- Command interval mean / P50 / P95 / max: 17.590 / 16.040 / 22.871 / 238.142 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 74.863 / 66.667 / 116.667 / 133.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.544 / 2.339 / 7.011 / 237.996 ms (steady clock)
- DDS publish call mean / P95 / max: 0.059 / 0.241 / 1.401 ms (steady clock)

#### Command bridge — final_v16_sweep_ego1p2_2

- Wall duration: 54.832 s
- Received / published control frames: 2306 / 2300
- Published control FPS (ROS simulation time / active wall time): 57.625 / 56.704 Hz
- Command stamp interval mean / P50 / P95 / max: 17.354 / 16.667 / 16.667 / 250.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.976 / 0.000 / 0.000 / 250.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.590 / 16.041 / 22.810 / 238.082 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.088 / 0.084 / 0.111 / 1.206 ms (steady clock)
