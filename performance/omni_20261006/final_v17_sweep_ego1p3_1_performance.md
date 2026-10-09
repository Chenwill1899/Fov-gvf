
### Run final_v17_sweep_ego1p3_1

- Started: 2026-10-07 00:15:56 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v17_sweep_ego1p3_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 41.989 / 40.017 s
- Applied simulation control frames: 2403
- Runtime FPS (simulation time / wall time): 60.050 / 57.230 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.340 / 15.872 / 18.085 / 79.649 ms
- Distinct applied command values: 1845
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.522 / 16.470 / 33.183 / 886.054 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — final_v17_sweep_ego1p3_1

- Wall duration: 55.945 s
- Timer callbacks / command frames / guidance frames: 2401 / 2400 / 2219
- Command FPS (ROS simulation time / active wall time): 59.975 / 57.652 Hz
- Guidance FPS (active wall time): 57.163 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.674 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.410 / 0.221 / 1.314 / 13.824 ms
- Command interval mean / P50 / P95 / max: 17.345 / 15.991 / 22.697 / 84.743 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 43.090 / 33.333 / 83.333 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.516 / 1.055 / 7.159 / 17.131 ms (steady clock)
- DDS publish call mean / P95 / max: 0.062 / 0.221 / 1.966 ms (steady clock)

#### Command bridge — final_v17_sweep_ego1p3_1

- Wall duration: 55.839 s
- Received / published control frames: 2400 / 2400
- Published control FPS (ROS simulation time / active wall time): 59.975 / 57.652 Hz
- Command stamp interval mean / P50 / P95 / max: 16.674 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.014 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.346 / 15.965 / 22.655 / 84.784 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.085 / 0.082 / 0.107 / 0.696 ms (steady clock)
