
### Run p5_operator_final_jitter_on_1

- Started: 2026-10-07 16:59:05 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_operator_final_jitter_on_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 36.955 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 55.284 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.927 / 16.703 / 18.953 / 81.229 ms
- Distinct applied command values: 1193
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 28.939 / 17.853 / 36.209 / 2070.220 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — p5_operator_final_jitter_on_1

- Wall duration: 50.806 s
- Received / published control frames: 2041 / 2041
- Published control FPS (ROS simulation time / active wall time): 60.000 / 55.807 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.310 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.919 / 16.518 / 32.869 / 97.362 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.085 / 0.082 / 0.111 / 1.685 ms (steady clock)

#### Controller — p5_operator_final_jitter_on_1

- Wall duration: 50.913 s
- Timer callbacks / command frames / guidance frames: 2042 / 2041 / 1620
- Command FPS (ROS simulation time / active wall time): 60.000 / 55.807 Hz
- Guidance FPS (active wall time): 48.267 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.296 / 0.216 / 0.294 / 4.242 ms
- Command interval mean / P50 / P95 / max: 17.919 / 16.508 / 32.836 / 97.270 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 57.377 / 50.000 / 100.000 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.535 / 0.374 / 14.939 / 23.443 ms (steady clock)
- DDS publish call mean / P95 / max: 0.054 / 0.078 / 2.286 ms (steady clock)
