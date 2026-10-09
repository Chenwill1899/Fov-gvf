
### Run candidate_v13_spin_ego1p3_1

- Started: 2026-10-06 23:28:41 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — candidate_v13_spin_ego1p3_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 67.283 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 57.117 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.367 / 15.988 / 18.253 / 79.245 ms
- Distinct applied command values: 3086
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 20.360 / 16.217 / 32.189 / 4224.795 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — candidate_v13_spin_ego1p3_1

- Wall duration: 81.358 s
- Timer callbacks / command frames / guidance frames: 3842 / 3841 / 3299
- Command FPS (ROS simulation time / active wall time): 60.000 / 57.584 Hz
- Guidance FPS (active wall time): 53.256 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 2.596 / 2.786 / 6.650 / 15.222 ms
- Command interval mean / P50 / P95 / max: 17.366 / 15.859 / 24.075 / 86.498 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 41.957 / 33.333 / 83.333 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 4.399 / 3.853 / 10.930 / 20.616 ms (steady clock)
- DDS publish call mean / P95 / max: 0.061 / 0.381 / 1.680 ms (steady clock)

#### Command bridge — candidate_v13_spin_ego1p3_1

- Wall duration: 81.249 s
- Received / published control frames: 3841 / 3841
- Published control FPS (ROS simulation time / active wall time): 60.000 / 57.584 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.135 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.366 / 15.849 / 24.046 / 86.577 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.086 / 0.083 / 0.110 / 1.467 ms (steady clock)
