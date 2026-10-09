
### Run p5_refine_execution_jitter_20261007_jitter_before_2

- Started: 2026-10-07 22:18:34 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_refine_execution_jitter_20261007_jitter_before_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 36.832 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 55.468 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.869 / 16.876 / 19.189 / 80.784 ms
- Distinct applied command values: 1361
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 25.313 / 17.474 / 35.195 / 2094.233 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_refine_execution_jitter_20261007_jitter_before_2

- Wall duration: 50.759 s
- Timer callbacks / command frames / guidance frames: 2042 / 2041 / 1620
- Command FPS (ROS simulation time / active wall time): 60.000 / 55.965 Hz
- Guidance FPS (active wall time): 48.368 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.282 / 0.209 / 0.319 / 4.014 ms
- Command interval mean / P50 / P95 / max: 17.868 / 16.616 / 24.075 / 84.825 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 39.218 / 33.333 / 83.333 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.718 / 2.719 / 7.059 / 13.658 ms (steady clock)
- DDS publish call mean / P95 / max: 0.054 / 0.074 / 2.223 ms (steady clock)

#### Command bridge — p5_refine_execution_jitter_20261007_jitter_before_2

- Wall duration: 50.647 s
- Received / published control frames: 2041 / 2041
- Published control FPS (ROS simulation time / active wall time): 60.000 / 55.966 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.868 / 16.650 / 24.051 / 84.737 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.088 / 0.084 / 0.110 / 1.430 ms (steady clock)
