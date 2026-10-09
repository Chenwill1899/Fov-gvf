
### Run candidate_v8_spin_ego1p3_3

- Started: 2026-10-06 23:03:27 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — candidate_v8_spin_ego1p3_3

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 70.580 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 54.449 Hz
- Isaac frame interval mean / P50 / P95 / max: 18.211 / 16.666 / 20.708 / 160.599 ms
- Distinct applied command values: 3072
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.574 / 16.944 / 34.186 / 4150.446 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — candidate_v8_spin_ego1p3_3

- Wall duration: 84.345 s
- Timer callbacks / command frames / guidance frames: 3842 / 3841 / 3300
- Command FPS (ROS simulation time / active wall time): 60.000 / 54.925 Hz
- Guidance FPS (active wall time): 50.562 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 2.477 / 2.834 / 5.970 / 18.401 ms
- Command interval mean / P50 / P95 / max: 18.207 / 16.924 / 23.969 / 128.663 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 38.434 / 33.333 / 66.667 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 4.291 / 3.989 / 9.677 / 47.479 ms (steady clock)
- DDS publish call mean / P95 / max: 0.080 / 0.377 / 10.137 ms (steady clock)

#### Command bridge — candidate_v8_spin_ego1p3_3

- Wall duration: 84.237 s
- Received / published control frames: 3841 / 3841
- Published control FPS (ROS simulation time / active wall time): 60.000 / 54.924 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.035 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.207 / 16.905 / 23.993 / 128.549 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.093 / 0.087 / 0.124 / 2.337 ms (steady clock)
