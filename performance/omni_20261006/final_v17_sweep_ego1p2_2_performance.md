
### Run final_v17_sweep_ego1p2_2

- Started: 2026-10-07 00:17:49 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v17_sweep_ego1p2_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 41.886 / 40.017 s
- Applied simulation control frames: 2403
- Runtime FPS (simulation time / wall time): 60.050 / 57.369 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.295 / 16.039 / 18.237 / 78.901 ms
- Distinct applied command values: 1725
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.923 / 16.609 / 34.841 / 1013.266 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — final_v17_sweep_ego1p2_2

- Wall duration: 55.844 s
- Timer callbacks / command frames / guidance frames: 2329 / 2328 / 2147
- Command FPS (ROS simulation time / active wall time): 58.175 / 56.061 Hz
- Guidance FPS (active wall time): 55.729 Hz
- Command interval in ROS time mean / P50 / P95 / max: 17.190 / 16.667 / 16.667 / 216.667 ms
- Avoidance compute mean / P50 / P95 / max: 1.337 / 0.395 / 1.658 / 182.810 ms
- Command interval mean / P50 / P95 / max: 17.838 / 15.927 / 24.919 / 197.139 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 69.073 / 66.667 / 116.667 / 133.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.336 / 0.854 / 8.269 / 197.062 ms (steady clock)
- DDS publish call mean / P95 / max: 0.056 / 0.092 / 1.542 ms (steady clock)

#### Command bridge — final_v17_sweep_ego1p2_2

- Wall duration: 55.738 s
- Received / published control frames: 2328 / 2325
- Published control FPS (ROS simulation time / active wall time): 58.175 / 55.990 Hz
- Command stamp interval mean / P50 / P95 / max: 17.190 / 16.667 / 16.667 / 216.667 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.816 / 0.000 / 0.000 / 216.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.837 / 15.940 / 24.976 / 197.110 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.089 / 0.084 / 0.117 / 1.334 ms (steady clock)
