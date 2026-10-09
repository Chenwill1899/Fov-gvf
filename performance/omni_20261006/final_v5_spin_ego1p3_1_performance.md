
### Run final_v5_spin_ego1p3_1

- Started: 2026-10-06 22:36:32 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v5_spin_ego1p3_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 67.373 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 57.040 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.392 / 16.044 / 18.451 / 80.750 ms
- Distinct applied command values: 2948
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.366 / 16.373 / 32.781 / 3939.581 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — final_v5_spin_ego1p3_1

- Wall duration: 81.130 s
- Timer callbacks / command frames / guidance frames: 3841 / 3840 / 3299
- Command FPS (ROS simulation time / active wall time): 59.984 / 57.495 Hz
- Guidance FPS (active wall time): 53.180 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.671 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 2.250 / 2.715 / 5.538 / 12.821 ms
- Command interval mean / P50 / P95 / max: 17.393 / 15.869 / 26.341 / 88.110 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 50.500 / 50.000 / 100.000 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.834 / 3.415 / 11.538 / 19.018 ms (steady clock)
- DDS publish call mean / P95 / max: 0.061 / 0.337 / 2.196 ms (steady clock)

#### Command bridge — final_v5_spin_ego1p3_1

- Wall duration: 81.020 s
- Received / published control frames: 3840 / 3840
- Published control FPS (ROS simulation time / active wall time): 59.984 / 57.496 Hz
- Command stamp interval mean / P50 / P95 / max: 16.671 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.030 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.393 / 15.857 / 26.360 / 88.254 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.083 / 0.080 / 0.108 / 0.819 ms (steady clock)
