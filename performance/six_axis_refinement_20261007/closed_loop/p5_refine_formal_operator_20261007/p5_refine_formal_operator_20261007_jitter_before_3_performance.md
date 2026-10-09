
### Run p5_refine_formal_operator_20261007_jitter_before_3

- Started: 2026-10-07 21:51:05 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_refine_formal_operator_20261007_jitter_before_3

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 37.216 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 54.895 Hz
- Isaac frame interval mean / P50 / P95 / max: 18.058 / 16.893 / 19.011 / 79.998 ms
- Distinct applied command values: 1347
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 25.771 / 17.511 / 35.390 / 2207.879 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_refine_formal_operator_20261007_jitter_before_3

- Wall duration: 51.122 s
- Timer callbacks / command frames / guidance frames: 2042 / 2042 / 1620
- Command FPS (ROS simulation time / active wall time): 60.000 / 54.864 Hz
- Guidance FPS (active wall time): 47.988 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.299 / 0.206 / 0.528 / 3.516 ms
- Command interval mean / P50 / P95 / max: 18.227 / 16.500 / 24.391 / 365.587 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 37.973 / 33.333 / 66.667 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.738 / 2.604 / 6.832 / 13.595 ms (steady clock)
- DDS publish call mean / P95 / max: 0.049 / 0.065 / 1.991 ms (steady clock)

#### Command bridge — p5_refine_formal_operator_20261007_jitter_before_3

- Wall duration: 51.011 s
- Received / published control frames: 2042 / 2042
- Published control FPS (ROS simulation time / active wall time): 60.000 / 54.864 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.227 / 16.487 / 24.319 / 365.855 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.086 / 0.084 / 0.106 / 1.107 ms (steady clock)
