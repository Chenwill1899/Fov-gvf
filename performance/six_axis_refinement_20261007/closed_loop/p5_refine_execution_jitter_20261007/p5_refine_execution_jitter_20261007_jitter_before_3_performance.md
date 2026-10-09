
### Run p5_refine_execution_jitter_20261007_jitter_before_3

- Started: 2026-10-07 22:20:58 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_refine_execution_jitter_20261007_jitter_before_3

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 38.253 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 53.407 Hz
- Isaac frame interval mean / P50 / P95 / max: 18.566 / 16.905 / 19.454 / 81.029 ms
- Distinct applied command values: 1282
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 27.871 / 17.724 / 36.586 / 2235.325 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — p5_refine_execution_jitter_20261007_jitter_before_3

- Wall duration: 52.102 s
- Received / published control frames: 2040 / 2040
- Published control FPS (ROS simulation time / active wall time): 59.971 / 53.829 Hz
- Command stamp interval mean / P50 / P95 / max: 16.675 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.577 / 16.547 / 28.851 / 91.638 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.090 / 0.086 / 0.112 / 1.022 ms (steady clock)

#### Controller — p5_refine_execution_jitter_20261007_jitter_before_3

- Wall duration: 52.212 s
- Timer callbacks / command frames / guidance frames: 2041 / 2040 / 1619
- Command FPS (ROS simulation time / active wall time): 59.971 / 53.829 Hz
- Guidance FPS (active wall time): 46.567 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.675 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.309 / 0.211 / 0.634 / 4.495 ms
- Command interval mean / P50 / P95 / max: 18.577 / 16.575 / 28.843 / 91.835 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 49.949 / 50.000 / 83.333 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.660 / 0.401 / 9.841 / 15.794 ms (steady clock)
- DDS publish call mean / P95 / max: 0.054 / 0.080 / 0.969 ms (steady clock)
