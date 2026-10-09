
### Run candidate_v20_sweep_ego1p3_1

- Started: 2026-10-07 01:07:34 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — candidate_v20_sweep_ego1p3_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 42.002 / 40.017 s
- Applied simulation control frames: 2403
- Runtime FPS (simulation time / wall time): 60.050 / 57.212 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.346 / 15.889 / 17.875 / 78.833 ms
- Distinct applied command values: 1848
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.466 / 16.537 / 33.269 / 887.104 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — candidate_v20_sweep_ego1p3_1

- Wall duration: 55.834 s
- Received / published control frames: 2398 / 2398
- Published control FPS (ROS simulation time / active wall time): 59.925 / 57.572 Hz
- Command stamp interval mean / P50 / P95 / max: 16.688 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.370 / 15.768 / 23.337 / 87.638 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.089 / 0.085 / 0.110 / 0.565 ms (steady clock)

#### Controller — candidate_v20_sweep_ego1p3_1

- Wall duration: 55.942 s
- Timer callbacks / command frames / guidance frames: 2399 / 2398 / 2217
- Command FPS (ROS simulation time / active wall time): 59.925 / 57.571 Hz
- Guidance FPS (active wall time): 57.174 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.688 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.323 / 0.224 / 0.834 / 7.017 ms
- Command interval mean / P50 / P95 / max: 17.370 / 15.741 / 23.430 / 87.477 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 39.535 / 33.333 / 66.667 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.385 / 1.113 / 7.310 / 13.894 ms (steady clock)
- DDS publish call mean / P95 / max: 0.063 / 0.353 / 1.812 ms (steady clock)
