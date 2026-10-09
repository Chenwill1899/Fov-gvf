
### Run final_v16_sweep_ego1p3_2

- Started: 2026-10-07 00:05:47 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v16_sweep_ego1p3_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 41.978 / 40.017 s
- Applied simulation control frames: 2403
- Runtime FPS (simulation time / wall time): 60.050 / 57.245 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.327 / 15.975 / 18.141 / 87.167 ms
- Distinct applied command values: 1671
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 23.782 / 16.705 / 33.738 / 1289.836 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — final_v16_sweep_ego1p3_2

- Wall duration: 55.942 s
- Timer callbacks / command frames / guidance frames: 2207 / 2206 / 2025
- Command FPS (ROS simulation time / active wall time): 55.125 / 53.028 Hz
- Guidance FPS (active wall time): 52.354 Hz
- Command interval in ROS time mean / P50 / P95 / max: 18.141 / 16.667 / 16.667 / 183.333 ms
- Avoidance compute mean / P50 / P95 / max: 3.153 / 0.234 / 14.645 / 159.312 ms
- Command interval mean / P50 / P95 / max: 18.858 / 16.173 / 38.529 / 166.896 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 44.272 / 33.333 / 83.333 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 5.059 / 2.447 / 15.574 / 166.790 ms (steady clock)
- DDS publish call mean / P95 / max: 0.064 / 0.403 / 1.436 ms (steady clock)

#### Command bridge — final_v16_sweep_ego1p3_2

- Wall duration: 55.833 s
- Received / published control frames: 2206 / 2198
- Published control FPS (ROS simulation time / active wall time): 55.125 / 52.836 Hz
- Command stamp interval mean / P50 / P95 / max: 18.141 / 16.667 / 16.667 / 183.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 2.327 / 0.000 / 16.667 / 183.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.858 / 16.174 / 38.711 / 166.880 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.091 / 0.086 / 0.116 / 1.084 ms (steady clock)
