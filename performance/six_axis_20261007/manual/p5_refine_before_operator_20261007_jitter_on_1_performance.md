
### Run p5_refine_before_operator_20261007_jitter_on_1

- Started: 2026-10-07 21:28:51 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_refine_before_operator_20261007_jitter_on_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 39.299 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 51.986 Hz
- Isaac frame interval mean / P50 / P95 / max: 19.076 / 17.681 / 20.971 / 81.607 ms
- Distinct applied command values: 1340
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 27.541 / 18.345 / 39.445 / 2312.992 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_refine_before_operator_20261007_jitter_on_1

- Wall duration: 53.665 s
- Timer callbacks / command frames / guidance frames: 2041 / 2040 / 1619
- Command FPS (ROS simulation time / active wall time): 59.971 / 52.397 Hz
- Guidance FPS (active wall time): 45.031 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.675 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.341 / 0.213 / 1.995 / 3.726 ms
- Command interval mean / P50 / P95 / max: 19.085 / 17.223 / 27.146 / 92.774 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 42.331 / 33.333 / 83.333 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.828 / 2.313 / 8.318 / 14.198 ms (steady clock)
- DDS publish call mean / P95 / max: 0.056 / 0.081 / 1.706 ms (steady clock)

#### Command bridge — p5_refine_before_operator_20261007_jitter_on_1

- Wall duration: 53.556 s
- Received / published control frames: 2040 / 2040
- Published control FPS (ROS simulation time / active wall time): 59.971 / 52.398 Hz
- Command stamp interval mean / P50 / P95 / max: 16.675 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 19.085 / 17.264 / 27.020 / 92.738 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.093 / 0.090 / 0.117 / 0.584 ms (steady clock)
