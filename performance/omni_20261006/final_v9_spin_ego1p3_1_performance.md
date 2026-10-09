
### Run final_v9_spin_ego1p3_1

- Started: 2026-10-06 23:08:28 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v9_spin_ego1p3_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 69.624 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 55.196 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.977 / 16.386 / 19.512 / 80.625 ms
- Distinct applied command values: 3033
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.402 / 16.646 / 33.051 / 4276.603 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — final_v9_spin_ego1p3_1

- Wall duration: 83.868 s
- Timer callbacks / command frames / guidance frames: 3810 / 3809 / 3268
- Command FPS (ROS simulation time / active wall time): 59.500 / 55.162 Hz
- Guidance FPS (active wall time): 51.047 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.807 / 16.667 / 16.667 / 66.667 ms
- Avoidance compute mean / P50 / P95 / max: 3.214 / 2.913 / 8.193 / 67.600 ms
- Command interval mean / P50 / P95 / max: 18.128 / 16.419 / 25.135 / 90.623 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 45.022 / 33.333 / 100.000 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 4.950 / 4.203 / 12.810 / 74.970 ms (steady clock)
- DDS publish call mean / P95 / max: 0.061 / 0.117 / 5.363 ms (steady clock)

#### Command bridge — final_v9_spin_ego1p3_1

- Wall duration: 83.759 s
- Received / published control frames: 3809 / 3809
- Published control FPS (ROS simulation time / active wall time): 59.500 / 55.163 Hz
- Command stamp interval mean / P50 / P95 / max: 16.807 / 16.667 / 16.667 / 66.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.486 / 0.000 / 0.000 / 66.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.128 / 16.422 / 25.054 / 90.652 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.090 / 0.087 / 0.119 / 0.458 ms (steady clock)
