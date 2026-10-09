
### Run p5_operator_v2_jitter_off_1

- Started: 2026-10-07 17:19:33 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_operator_v2_jitter_off_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 37.544 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 54.417 Hz
- Isaac frame interval mean / P50 / P95 / max: 18.219 / 16.597 / 19.039 / 79.861 ms
- Distinct applied command values: 1324
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 26.632 / 17.249 / 35.778 / 2298.509 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — p5_operator_v2_jitter_off_1

- Wall duration: 51.686 s
- Received / published control frames: 2041 / 2041
- Published control FPS (ROS simulation time / active wall time): 60.000 / 54.891 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.218 / 16.033 / 26.122 / 92.327 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.086 / 0.083 / 0.107 / 1.250 ms (steady clock)

#### Controller — p5_operator_v2_jitter_off_1

- Wall duration: 51.794 s
- Timer callbacks / command frames / guidance frames: 2042 / 2041 / 1620
- Command FPS (ROS simulation time / active wall time): 60.000 / 54.891 Hz
- Guidance FPS (active wall time): 47.208 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.321 / 0.209 / 1.983 / 3.046 ms
- Command interval mean / P50 / P95 / max: 18.218 / 16.009 / 26.194 / 92.354 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 42.675 / 33.333 / 83.333 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.679 / 0.495 / 8.529 / 14.787 ms (steady clock)
- DDS publish call mean / P95 / max: 0.047 / 0.065 / 0.978 ms (steady clock)
