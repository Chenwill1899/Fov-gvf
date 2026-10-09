
### Run final_v5_sweep_ego1p2_2

- Started: 2026-10-06 22:32:19 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v5_sweep_ego1p2_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 42.044 / 40.017 s
- Applied simulation control frames: 2403
- Runtime FPS (simulation time / wall time): 60.050 / 57.154 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.360 / 15.932 / 18.097 / 79.018 ms
- Distinct applied command values: 1820
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.940 / 16.426 / 34.201 / 962.851 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — final_v5_sweep_ego1p2_2

- Wall duration: 56.068 s
- Timer callbacks / command frames / guidance frames: 2343 / 2342 / 2161
- Command FPS (ROS simulation time / active wall time): 58.525 / 56.191 Hz
- Guidance FPS (active wall time): 55.453 Hz
- Command interval in ROS time mean / P50 / P95 / max: 17.087 / 16.667 / 16.667 / 133.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.444 / 0.412 / 2.556 / 124.605 ms
- Command interval mean / P50 / P95 / max: 17.797 / 15.963 / 24.531 / 133.783 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 67.153 / 66.667 / 116.667 / 133.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.574 / 2.490 / 8.289 / 133.306 ms (steady clock)
- DDS publish call mean / P95 / max: 0.062 / 0.375 / 1.880 ms (steady clock)

#### Command bridge — final_v5_sweep_ego1p2_2

- Wall duration: 56.058 s
- Received / published control frames: 2342 / 2341
- Published control FPS (ROS simulation time / active wall time): 58.525 / 56.167 Hz
- Command stamp interval mean / P50 / P95 / max: 17.087 / 16.667 / 16.667 / 133.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.719 / 0.000 / 0.000 / 133.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.796 / 15.960 / 24.323 / 133.778 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.087 / 0.085 / 0.108 / 1.382 ms (steady clock)
