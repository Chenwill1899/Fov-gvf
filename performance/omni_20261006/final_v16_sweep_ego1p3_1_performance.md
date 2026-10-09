
### Run final_v16_sweep_ego1p3_1

- Started: 2026-10-07 00:04:50 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v16_sweep_ego1p3_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 42.049 / 40.017 s
- Applied simulation control frames: 2403
- Runtime FPS (simulation time / wall time): 60.050 / 57.148 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.360 / 16.001 / 18.610 / 79.656 ms
- Distinct applied command values: 1656
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 24.025 / 16.612 / 35.400 / 1225.193 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — final_v16_sweep_ego1p3_1

- Wall duration: 56.124 s
- Timer callbacks / command frames / guidance frames: 2161 / 2160 / 1979
- Command FPS (ROS simulation time / active wall time): 53.975 / 51.842 Hz
- Guidance FPS (active wall time): 51.307 Hz
- Command interval in ROS time mean / P50 / P95 / max: 18.527 / 16.667 / 16.667 / 583.333 ms
- Avoidance compute mean / P50 / P95 / max: 3.207 / 0.231 / 7.656 / 575.059 ms
- Command interval mean / P50 / P95 / max: 19.290 / 16.094 / 30.457 / 579.990 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 45.461 / 33.333 / 100.000 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 5.127 / 1.219 / 11.768 / 579.921 ms (steady clock)
- DDS publish call mean / P95 / max: 0.056 / 0.086 / 1.469 ms (steady clock)

#### Command bridge — final_v16_sweep_ego1p3_1

- Wall duration: 56.014 s
- Received / published control frames: 2160 / 2152
- Published control FPS (ROS simulation time / active wall time): 53.975 / 51.650 Hz
- Command stamp interval mean / P50 / P95 / max: 18.527 / 16.667 / 16.667 / 583.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 2.539 / 0.000 / 0.000 / 583.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 19.290 / 16.060 / 31.104 / 579.956 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.089 / 0.083 / 0.112 / 2.945 ms (steady clock)
