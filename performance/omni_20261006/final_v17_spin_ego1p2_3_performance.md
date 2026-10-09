
### Run final_v17_spin_ego1p2_3

- Started: 2026-10-07 00:26:15 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v17_spin_ego1p2_3

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 68.666 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 55.967 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.729 / 16.182 / 18.616 / 93.753 ms
- Distinct applied command values: 2979
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.619 / 16.533 / 33.388 / 4126.592 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — final_v17_spin_ego1p2_3

- Wall duration: 82.375 s
- Received / published control frames: 3838 / 3838
- Published control FPS (ROS simulation time / active wall time): 59.953 / 56.358 Hz
- Command stamp interval mean / P50 / P95 / max: 16.680 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.744 / 16.038 / 23.763 / 86.321 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.092 / 0.088 / 0.116 / 2.606 ms (steady clock)

#### Controller — final_v17_spin_ego1p2_3

- Wall duration: 82.478 s
- Timer callbacks / command frames / guidance frames: 3839 / 3838 / 3297
- Command FPS (ROS simulation time / active wall time): 59.953 / 56.358 Hz
- Guidance FPS (active wall time): 51.959 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.680 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.630 / 0.538 / 1.096 / 8.564 ms
- Command interval mean / P50 / P95 / max: 17.744 / 16.022 / 23.800 / 86.228 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 72.035 / 66.667 / 116.667 / 133.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.828 / 2.683 / 7.274 / 15.608 ms (steady clock)
- DDS publish call mean / P95 / max: 0.056 / 0.094 / 1.635 ms (steady clock)
