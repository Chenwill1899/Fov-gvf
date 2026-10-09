
### Run final_v22_recovery_ego1p3_1

- Started: 2026-10-07 02:12:23 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v22_recovery_ego1p3_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 73.705 / 72.017 s
- Applied simulation control frames: 4323
- Runtime FPS (simulation time / wall time): 60.028 / 58.653 Hz
- Isaac frame interval mean / P50 / P95 / max: 16.926 / 15.635 / 17.800 / 78.937 ms
- Distinct applied command values: 2148
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 32.240 / 16.165 / 32.550 / 5669.104 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — final_v22_recovery_ego1p3_1

- Wall duration: 87.567 s
- Timer callbacks / command frames / guidance frames: 4312 / 4311 / 3232
- Command FPS (ROS simulation time / active wall time): 59.861 / 58.947 Hz
- Guidance FPS (active wall time): 47.279 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.705 / 16.667 / 16.667 / 50.000 ms
- Avoidance compute mean / P50 / P95 / max: 1.123 / 0.390 / 4.759 / 34.064 ms
- Command interval mean / P50 / P95 / max: 16.964 / 15.557 / 23.719 / 89.153 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 38.511 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.122 / 2.859 / 7.799 / 45.268 ms (steady clock)
- DDS publish call mean / P95 / max: 0.054 / 0.128 / 1.183 ms (steady clock)

#### Command bridge — final_v22_recovery_ego1p3_1

- Wall duration: 87.457 s
- Received / published control frames: 4311 / 4311
- Published control FPS (ROS simulation time / active wall time): 59.861 / 58.947 Hz
- Command stamp interval mean / P50 / P95 / max: 16.705 / 16.667 / 16.667 / 50.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.104 / 0.000 / 0.000 / 50.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 16.964 / 15.571 / 23.682 / 89.116 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.086 / 0.083 / 0.107 / 1.277 ms (steady clock)
