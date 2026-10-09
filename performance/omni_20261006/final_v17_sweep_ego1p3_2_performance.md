
### Run final_v17_sweep_ego1p3_2

- Started: 2026-10-07 00:16:53 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v17_sweep_ego1p3_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 41.530 / 40.017 s
- Applied simulation control frames: 2403
- Runtime FPS (simulation time / wall time): 60.050 / 57.861 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.144 / 15.747 / 17.920 / 78.221 ms
- Distinct applied command values: 1840
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.339 / 16.397 / 32.843 / 1012.134 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — final_v17_sweep_ego1p3_2

- Wall duration: 55.534 s
- Timer callbacks / command frames / guidance frames: 2400 / 2399 / 2218
- Command FPS (ROS simulation time / active wall time): 59.950 / 58.285 Hz
- Guidance FPS (active wall time): 57.989 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.681 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.297 / 0.223 / 0.735 / 6.619 ms
- Command interval mean / P50 / P95 / max: 17.157 / 15.763 / 22.975 / 83.852 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 39.157 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.365 / 1.420 / 6.710 / 14.436 ms (steady clock)
- DDS publish call mean / P95 / max: 0.058 / 0.142 / 2.018 ms (steady clock)

#### Command bridge — final_v17_sweep_ego1p3_2

- Wall duration: 55.429 s
- Received / published control frames: 2399 / 2399
- Published control FPS (ROS simulation time / active wall time): 59.950 / 58.286 Hz
- Command stamp interval mean / P50 / P95 / max: 16.681 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.157 / 15.745 / 22.896 / 83.865 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.088 / 0.084 / 0.110 / 1.136 ms (steady clock)
