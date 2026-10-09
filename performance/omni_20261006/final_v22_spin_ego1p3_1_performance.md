
### Run final_v22_spin_ego1p3_1

- Started: 2026-10-07 02:04:07 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v22_spin_ego1p3_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 66.406 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 57.872 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.143 / 15.883 / 18.097 / 81.346 ms
- Distinct applied command values: 2877
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.537 / 16.380 / 32.789 / 4099.046 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — final_v22_spin_ego1p3_1

- Wall duration: 80.158 s
- Timer callbacks / command frames / guidance frames: 3809 / 3809 / 3267
- Command FPS (ROS simulation time / active wall time): 59.485 / 57.352 Hz
- Guidance FPS (active wall time): 53.538 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.811 / 16.667 / 16.667 / 100.000 ms
- Avoidance compute mean / P50 / P95 / max: 1.383 / 0.419 / 6.171 / 77.949 ms
- Command interval mean / P50 / P95 / max: 17.436 / 16.066 / 23.118 / 565.776 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 39.578 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.434 / 3.026 / 8.882 / 86.942 ms (steady clock)
- DDS publish call mean / P95 / max: 0.058 / 0.203 / 1.944 ms (steady clock)

#### Command bridge — final_v22_spin_ego1p3_1

- Wall duration: 80.050 s
- Received / published control frames: 3809 / 3809
- Published control FPS (ROS simulation time / active wall time): 59.485 / 57.352 Hz
- Command stamp interval mean / P50 / P95 / max: 16.811 / 16.667 / 16.667 / 100.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.376 / 0.000 / 0.000 / 100.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.436 / 16.044 / 23.074 / 565.997 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.087 / 0.083 / 0.112 / 1.662 ms (steady clock)
