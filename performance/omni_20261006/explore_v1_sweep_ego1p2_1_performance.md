
### Run explore_v1_sweep_ego1p2_1

- Started: 2026-10-06 22:06:25 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — explore_v1_sweep_ego1p2_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 41.561 / 40.017 s
- Applied simulation control frames: 2403
- Runtime FPS (simulation time / wall time): 60.050 / 57.818 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.163 / 15.853 / 17.987 / 78.085 ms
- Distinct applied command values: 1791
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.872 / 16.301 / 33.266 / 934.773 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — explore_v1_sweep_ego1p2_1

- Wall duration: 55.286 s
- Received / published control frames: 2333 / 2329
- Published control FPS (ROS simulation time / active wall time): 58.300 / 56.520 Hz
- Command stamp interval mean / P50 / P95 / max: 17.153 / 16.667 / 16.667 / 233.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.750 / 0.000 / 0.000 / 233.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.662 / 15.873 / 22.627 / 254.222 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.087 / 0.085 / 0.110 / 0.335 ms (steady clock)

#### Controller — explore_v1_sweep_ego1p2_1

- Wall duration: 55.391 s
- Timer callbacks / command frames / guidance frames: 2334 / 2333 / 2152
- Command FPS (ROS simulation time / active wall time): 58.300 / 56.617 Hz
- Guidance FPS (active wall time): 56.281 Hz
- Command interval in ROS time mean / P50 / P95 / max: 17.153 / 16.667 / 16.667 / 233.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.277 / 0.409 / 1.243 / 217.431 ms
- Command interval mean / P50 / P95 / max: 17.663 / 15.899 / 22.649 / 254.163 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 74.621 / 66.667 / 116.667 / 150.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.349 / 2.482 / 6.995 / 217.592 ms (steady clock)
- DDS publish call mean / P95 / max: 0.060 / 0.281 / 1.311 ms (steady clock)
