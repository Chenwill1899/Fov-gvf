
### Run final_v16_sweep_ego1p3_3

- Started: 2026-10-07 00:10:05 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v16_sweep_ego1p3_3

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 41.505 / 40.017 s
- Applied simulation control frames: 2403
- Runtime FPS (simulation time / wall time): 60.050 / 57.897 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.137 / 15.796 / 17.982 / 78.216 ms
- Distinct applied command values: 1785
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.950 / 16.310 / 32.852 / 1231.644 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — final_v16_sweep_ego1p3_3

- Wall duration: 55.450 s
- Timer callbacks / command frames / guidance frames: 2192 / 2191 / 2010
- Command FPS (ROS simulation time / active wall time): 54.750 / 53.250 Hz
- Guidance FPS (active wall time): 52.487 Hz
- Command interval in ROS time mean / P50 / P95 / max: 18.265 / 16.667 / 16.667 / 283.333 ms
- Avoidance compute mean / P50 / P95 / max: 3.299 / 0.231 / 11.649 / 258.792 ms
- Command interval mean / P50 / P95 / max: 18.779 / 16.162 / 32.256 / 269.914 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 37.521 / 33.333 / 66.667 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 5.256 / 3.103 / 13.625 / 269.807 ms (steady clock)
- DDS publish call mean / P95 / max: 0.066 / 0.417 / 1.412 ms (steady clock)

#### Command bridge — final_v16_sweep_ego1p3_3

- Wall duration: 55.342 s
- Received / published control frames: 2191 / 2183
- Published control FPS (ROS simulation time / active wall time): 54.750 / 53.056 Hz
- Command stamp interval mean / P50 / P95 / max: 18.265 / 16.667 / 16.667 / 283.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 2.373 / 0.000 / 0.000 / 283.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.779 / 16.151 / 32.279 / 269.823 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.089 / 0.085 / 0.114 / 1.190 ms (steady clock)
