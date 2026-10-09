
### Run p5_refine_execution_jitter_20261007_jitter_after_3

- Started: 2026-10-07 22:23:24 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_refine_execution_jitter_20261007_jitter_after_3

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 37.258 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 54.833 Hz
- Isaac frame interval mean / P50 / P95 / max: 18.046 / 16.896 / 19.083 / 78.936 ms
- Distinct applied command values: 1573
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.158 / 17.061 / 19.655 / 2100.726 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_refine_execution_jitter_20261007_jitter_after_3

- Wall duration: 51.584 s
- Timer callbacks / command frames / guidance frames: 2041 / 2040 / 1619
- Command FPS (ROS simulation time / active wall time): 59.971 / 55.389 Hz
- Guidance FPS (active wall time): 47.761 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.675 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.365 / 0.210 / 2.196 / 4.462 ms
- Command interval mean / P50 / P95 / max: 18.054 / 16.367 / 24.832 / 85.871 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 38.388 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.712 / 2.083 / 7.502 / 13.573 ms (steady clock)
- DDS publish call mean / P95 / max: 0.053 / 0.076 / 2.071 ms (steady clock)

#### Command bridge — p5_refine_execution_jitter_20261007_jitter_after_3

- Wall duration: 51.573 s
- Received / published control frames: 2040 / 2040
- Published control FPS (ROS simulation time / active wall time): 59.971 / 55.389 Hz
- Command stamp interval mean / P50 / P95 / max: 16.675 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.054 / 16.393 / 24.807 / 85.593 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.086 / 0.084 / 0.108 / 0.583 ms (steady clock)
