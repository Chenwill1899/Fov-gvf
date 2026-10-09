
### Run paper_parallel_field

- Started: 2026-09-24 17:53:20 CST
- Scene: `/home/starry/isaac-data/EGO1P1/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `84`
- Manual input: `trace`

#### Isaac command application — paper_parallel_field

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 31.761 / 29.017 s
- Applied simulation control frames: 1743
- Runtime FPS (simulation time / wall time): 60.069 / 54.879 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.894 / 16.356 / 19.997 / 79.349 ms
- Distinct applied command values: 584
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 48.285 / 29.916 / 94.218 / 3021.901 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — paper_parallel_field

- Wall duration: 46.063 s
- Timer callbacks / command frames / guidance frames: 1190 / 1189 / 1017
- Command FPS (ROS simulation time / active wall time): 40.966 / 38.119 Hz
- Guidance FPS (active wall time): 37.243 Hz
- Command interval in ROS time mean / P50 / P95 / max: 24.411 / 16.667 / 33.333 / 66.667 ms
- Avoidance compute mean / P50 / P95 / max: 3.975 / 0.078 / 19.552 / 34.076 ms
- Command interval mean / P50 / P95 / max: 26.234 / 22.945 / 48.786 / 123.153 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 23.091 / 16.667 / 33.333 / 50.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 21.429 / 21.671 / 40.091 / 59.316 ms (steady clock)
- DDS publish call mean / P95 / max: 0.085 / 0.507 / 2.713 ms (steady clock)

#### Command bridge — paper_parallel_field

- Wall duration: 45.614 s
- Received / published control frames: 1189 / 1189
- Published control FPS (ROS simulation time / active wall time): 40.966 / 38.118 Hz
- Command stamp interval mean / P50 / P95 / max: 24.411 / 16.667 / 33.333 / 66.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 18.755 / 16.667 / 33.333 / 66.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 26.234 / 22.928 / 48.825 / 123.298 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.126 / 0.112 / 0.172 / 2.043 ms (steady clock)
