
### Run candidate_v8_spin_ego1p3_2

- Started: 2026-10-06 23:02:06 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — candidate_v8_spin_ego1p3_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 66.979 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 57.376 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.287 / 15.983 / 18.132 / 79.110 ms
- Distinct applied command values: 2950
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.021 / 16.281 / 32.776 / 4320.252 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — candidate_v8_spin_ego1p3_2

- Wall duration: 80.750 s
- Timer callbacks / command frames / guidance frames: 3745 / 3744 / 3211
- Command FPS (ROS simulation time / active wall time): 58.484 / 56.387 Hz
- Guidance FPS (active wall time): 51.935 Hz
- Command interval in ROS time mean / P50 / P95 / max: 17.099 / 16.667 / 16.667 / 366.667 ms
- Avoidance compute mean / P50 / P95 / max: 3.313 / 2.812 / 9.315 / 345.778 ms
- Command interval mean / P50 / P95 / max: 17.734 / 15.940 / 24.102 / 348.830 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 40.761 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 5.227 / 4.225 / 13.301 / 348.756 ms (steady clock)
- DDS publish call mean / P95 / max: 0.055 / 0.119 / 1.046 ms (steady clock)

#### Command bridge — candidate_v8_spin_ego1p3_2

- Wall duration: 80.642 s
- Received / published control frames: 3744 / 3739
- Published control FPS (ROS simulation time / active wall time): 58.484 / 56.312 Hz
- Command stamp interval mean / P50 / P95 / max: 17.099 / 16.667 / 16.667 / 366.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 1.033 / 0.000 / 0.000 / 366.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.734 / 15.934 / 24.127 / 348.837 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.089 / 0.085 / 0.112 / 1.805 ms (steady clock)
