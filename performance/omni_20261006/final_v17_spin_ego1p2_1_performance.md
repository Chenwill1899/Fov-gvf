
### Run final_v17_spin_ego1p2_1

- Started: 2026-10-07 00:20:40 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v17_spin_ego1p2_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 67.180 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 57.204 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.342 / 15.841 / 17.925 / 80.071 ms
- Distinct applied command values: 3026
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 20.678 / 16.146 / 32.100 / 4261.525 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — final_v17_spin_ego1p2_1

- Wall duration: 80.962 s
- Received / published control frames: 3841 / 3841
- Published control FPS (ROS simulation time / active wall time): 59.984 / 57.171 Hz
- Command stamp interval mean / P50 / P95 / max: 16.671 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.013 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.491 / 15.968 / 23.477 / 588.377 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.086 / 0.083 / 0.111 / 1.131 ms (steady clock)

#### Controller — final_v17_spin_ego1p2_1

- Wall duration: 81.070 s
- Timer callbacks / command frames / guidance frames: 3841 / 3841 / 3299
- Command FPS (ROS simulation time / active wall time): 59.984 / 57.171 Hz
- Guidance FPS (active wall time): 53.524 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.671 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.615 / 0.509 / 1.206 / 8.763 ms
- Command interval mean / P50 / P95 / max: 17.492 / 15.988 / 23.528 / 588.484 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 70.496 / 66.667 / 116.667 / 133.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.716 / 1.270 / 7.696 / 17.896 ms (steady clock)
- DDS publish call mean / P95 / max: 0.058 / 0.184 / 1.487 ms (steady clock)
