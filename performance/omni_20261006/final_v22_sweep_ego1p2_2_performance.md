
### Run final_v22_sweep_ego1p2_2

- Started: 2026-10-07 01:59:38 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v22_sweep_ego1p2_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 41.670 / 40.017 s
- Applied simulation control frames: 2403
- Runtime FPS (simulation time / wall time): 60.050 / 57.668 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.202 / 15.985 / 18.263 / 80.188 ms
- Distinct applied command values: 1860
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.215 / 16.437 / 33.323 / 933.488 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — final_v22_sweep_ego1p2_2

- Wall duration: 55.910 s
- Timer callbacks / command frames / guidance frames: 2337 / 2337 / 2155
- Command FPS (ROS simulation time / active wall time): 58.376 / 56.074 Hz
- Guidance FPS (active wall time): 55.914 Hz
- Command interval in ROS time mean / P50 / P95 / max: 17.130 / 16.667 / 16.667 / 216.667 ms
- Avoidance compute mean / P50 / P95 / max: 1.411 / 0.414 / 2.338 / 209.693 ms
- Command interval mean / P50 / P95 / max: 17.834 / 15.904 / 22.814 / 373.360 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 74.671 / 66.667 / 116.667 / 133.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.541 / 2.793 / 7.100 / 219.679 ms (steady clock)
- DDS publish call mean / P95 / max: 0.064 / 0.392 / 1.390 ms (steady clock)

#### Command bridge — final_v22_sweep_ego1p2_2

- Wall duration: 55.900 s
- Received / published control frames: 2337 / 2335
- Published control FPS (ROS simulation time / active wall time): 58.376 / 56.026 Hz
- Command stamp interval mean / P50 / P95 / max: 17.130 / 16.667 / 16.667 / 216.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.749 / 0.000 / 0.000 / 216.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.834 / 15.911 / 22.792 / 373.926 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.091 / 0.087 / 0.118 / 0.825 ms (steady clock)
