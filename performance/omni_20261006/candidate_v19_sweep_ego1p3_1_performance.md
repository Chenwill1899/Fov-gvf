
### Run candidate_v19_sweep_ego1p3_1

- Started: 2026-10-07 01:03:40 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — candidate_v19_sweep_ego1p3_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 40.869 / 40.017 s
- Applied simulation control frames: 2403
- Runtime FPS (simulation time / wall time): 60.050 / 58.797 Hz
- Isaac frame interval mean / P50 / P95 / max: 16.871 / 15.784 / 17.798 / 77.800 ms
- Distinct applied command values: 1914
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 20.191 / 16.272 / 32.366 / 874.179 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — candidate_v19_sweep_ego1p3_1

- Wall duration: 55.137 s
- Received / published control frames: 2399 / 2399
- Published control FPS (ROS simulation time / active wall time): 59.950 / 59.226 Hz
- Command stamp interval mean / P50 / P95 / max: 16.681 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.007 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 16.884 / 15.868 / 21.894 / 85.155 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.089 / 0.086 / 0.113 / 0.354 ms (steady clock)

#### Controller — candidate_v19_sweep_ego1p3_1

- Wall duration: 55.244 s
- Timer callbacks / command frames / guidance frames: 2400 / 2399 / 2219
- Command FPS (ROS simulation time / active wall time): 59.950 / 59.226 Hz
- Guidance FPS (active wall time): 58.755 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.681 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.428 / 0.221 / 1.670 / 15.611 ms
- Command interval mean / P50 / P95 / max: 16.885 / 15.882 / 21.941 / 85.193 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 39.477 / 33.333 / 66.667 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.577 / 2.374 / 6.725 / 18.946 ms (steady clock)
- DDS publish call mean / P95 / max: 0.057 / 0.165 / 1.461 ms (steady clock)
