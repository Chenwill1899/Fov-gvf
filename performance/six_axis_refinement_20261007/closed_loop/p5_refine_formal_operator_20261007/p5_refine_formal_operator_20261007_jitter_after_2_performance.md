
### Run p5_refine_formal_operator_20261007_jitter_after_2

- Started: 2026-10-07 21:46:23 CST
- Scene: `/home/starry/isaac-data/EGO1P5/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — p5_refine_formal_operator_20261007_jitter_after_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 36.815 / 34.017 s
- Applied simulation control frames: 2043
- Runtime FPS (simulation time / wall time): 60.059 / 55.494 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.857 / 16.852 / 19.116 / 78.559 ms
- Distinct applied command values: 1260
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 27.326 / 17.663 / 35.900 / 2171.901 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — p5_refine_formal_operator_20261007_jitter_after_2

- Wall duration: 50.703 s
- Timer callbacks / command frames / guidance frames: 2042 / 2041 / 1620
- Command FPS (ROS simulation time / active wall time): 60.000 / 55.992 Hz
- Guidance FPS (active wall time): 48.304 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.340 / 0.212 / 2.116 / 3.708 ms
- Command interval mean / P50 / P95 / max: 17.860 / 16.678 / 28.209 / 84.894 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 43.858 / 33.333 / 83.333 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.613 / 0.391 / 10.891 / 14.012 ms (steady clock)
- DDS publish call mean / P95 / max: 0.053 / 0.073 / 2.583 ms (steady clock)

#### Command bridge — p5_refine_formal_operator_20261007_jitter_after_2

- Wall duration: 50.594 s
- Received / published control frames: 2041 / 2041
- Published control FPS (ROS simulation time / active wall time): 60.000 / 55.992 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.860 / 16.692 / 28.137 / 84.702 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.088 / 0.084 / 0.112 / 1.422 ms (steady clock)
