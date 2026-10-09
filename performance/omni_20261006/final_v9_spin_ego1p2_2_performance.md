
### Run final_v9_spin_ego1p2_2

- Started: 2026-10-06 23:11:16 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v9_spin_ego1p2_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 66.291 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 57.972 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.110 / 15.879 / 18.144 / 80.084 ms
- Distinct applied command values: 2847
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.699 / 16.374 / 32.729 / 4201.821 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — final_v9_spin_ego1p2_2

- Wall duration: 79.941 s
- Received / published control frames: 3841 / 3841
- Published control FPS (ROS simulation time / active wall time): 60.000 / 58.449 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.065 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.109 / 15.803 / 23.617 / 90.521 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.090 / 0.087 / 0.115 / 1.380 ms (steady clock)

#### Controller — final_v9_spin_ego1p2_2

- Wall duration: 80.048 s
- Timer callbacks / command frames / guidance frames: 3842 / 3841 / 3300
- Command FPS (ROS simulation time / active wall time): 60.000 / 58.448 Hz
- Guidance FPS (active wall time): 54.313 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.774 / 0.533 / 1.682 / 15.330 ms
- Command interval mean / P50 / P95 / max: 17.109 / 15.775 / 23.655 / 90.517 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 69.177 / 66.667 / 116.667 / 133.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.867 / 2.045 / 7.992 / 26.222 ms (steady clock)
- DDS publish call mean / P95 / max: 0.065 / 0.405 / 2.360 ms (steady clock)
