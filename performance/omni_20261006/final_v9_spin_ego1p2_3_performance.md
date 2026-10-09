
### Run final_v9_spin_ego1p2_3

- Started: 2026-10-06 23:12:37 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v9_spin_ego1p2_3

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 71.895 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 53.453 Hz
- Isaac frame interval mean / P50 / P95 / max: 18.570 / 16.713 / 24.387 / 87.633 ms
- Distinct applied command values: 2876
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 23.444 / 17.153 / 35.468 / 4621.085 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — final_v9_spin_ego1p2_3

- Wall duration: 85.652 s
- Timer callbacks / command frames / guidance frames: 3839 / 3839 / 3297
- Command FPS (ROS simulation time / active wall time): 59.953 / 53.393 Hz
- Guidance FPS (active wall time): 49.556 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.680 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.800 / 0.541 / 1.163 / 15.010 ms
- Command interval mean / P50 / P95 / max: 18.729 / 16.716 / 26.732 / 576.517 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 70.433 / 66.667 / 116.667 / 133.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.021 / 1.699 / 8.618 / 40.439 ms (steady clock)
- DDS publish call mean / P95 / max: 0.072 / 0.374 / 3.315 ms (steady clock)

#### Command bridge — final_v9_spin_ego1p2_3

- Wall duration: 85.546 s
- Received / published control frames: 3839 / 3839
- Published control FPS (ROS simulation time / active wall time): 59.953 / 53.393 Hz
- Command stamp interval mean / P50 / P95 / max: 16.680 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.117 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.729 / 16.754 / 26.548 / 577.102 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.100 / 0.090 / 0.132 / 3.926 ms (steady clock)
