
### Run final_v22_sweep_ego1p3_1

- Started: 2026-10-07 01:57:45 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v22_sweep_ego1p3_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 41.757 / 40.017 s
- Applied simulation control frames: 2403
- Runtime FPS (simulation time / wall time): 60.050 / 57.547 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.242 / 15.753 / 17.687 / 78.779 ms
- Distinct applied command values: 1845
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.371 / 16.330 / 32.662 / 995.008 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — final_v22_sweep_ego1p3_1

- Wall duration: 56.033 s
- Timer callbacks / command frames / guidance frames: 2402 / 2401 / 2220
- Command FPS (ROS simulation time / active wall time): 60.000 / 58.002 Hz
- Guidance FPS (active wall time): 57.760 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms
- Avoidance compute mean / P50 / P95 / max: 0.334 / 0.223 / 0.871 / 6.512 ms
- Command interval mean / P50 / P95 / max: 17.241 / 15.877 / 22.908 / 83.835 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 44.399 / 33.333 / 83.333 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.436 / 0.952 / 6.973 / 15.249 ms (steady clock)
- DDS publish call mean / P95 / max: 0.056 / 0.114 / 1.890 ms (steady clock)

#### Command bridge — final_v22_sweep_ego1p3_1

- Wall duration: 55.924 s
- Received / published control frames: 2401 / 2401
- Published control FPS (ROS simulation time / active wall time): 60.000 / 58.002 Hz
- Command stamp interval mean / P50 / P95 / max: 16.667 / 16.667 / 16.667 / 16.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.241 / 15.849 / 22.717 / 83.745 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.088 / 0.084 / 0.111 / 1.392 ms (steady clock)
