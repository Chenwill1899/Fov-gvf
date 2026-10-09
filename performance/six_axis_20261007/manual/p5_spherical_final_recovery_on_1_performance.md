
### Run p5_spherical_final_recovery_on_1

- Started: 2026-10-07 17:03:09 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_spherical_final_recovery_on_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 74.388 / 72.017 s
- Applied simulation control frames: 4323
- Runtime FPS (simulation time / wall time): 60.028 / 58.114 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.085 / 15.699 / 17.796 / 80.589 ms
- Distinct applied command values: 2082
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 33.593 / 16.252 / 33.913 / 5629.589 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_spherical_final_recovery_on_1

- Wall duration: 88.144 s
- Timer callbacks / command frames / guidance frames: 4309 / 4308 / 3228
- Command FPS (ROS simulation time / active wall time): 59.819 / 58.356 Hz
- Guidance FPS (active wall time): 46.758 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.717 / 16.667 / 16.667 / 66.667 ms
- Avoidance compute mean / P50 / P95 / max: 1.036 / 0.386 / 4.686 / 42.517 ms
- Command interval mean / P50 / P95 / max: 17.136 / 15.289 / 28.195 / 99.452 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 36.984 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 4.384 / 4.211 / 12.057 / 68.326 ms (steady clock)
- DDS publish call mean / P95 / max: 0.052 / 0.078 / 1.528 ms (steady clock)

#### Command bridge — p5_spherical_final_recovery_on_1

- Wall duration: 88.032 s
- Received / published control frames: 4308 / 4308
- Published control FPS (ROS simulation time / active wall time): 59.819 / 58.356 Hz
- Command stamp interval mean / P50 / P95 / max: 16.717 / 16.667 / 16.667 / 66.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.588 / 0.000 / 0.000 / 66.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.136 / 15.360 / 28.237 / 99.441 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.085 / 0.082 / 0.107 / 1.481 ms (steady clock)
