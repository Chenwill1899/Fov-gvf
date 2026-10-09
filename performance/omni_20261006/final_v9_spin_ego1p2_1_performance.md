
### Run final_v9_spin_ego1p2_1

- Started: 2026-10-06 23:07:06 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v9_spin_ego1p2_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 68.176 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 56.369 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.593 / 16.217 / 18.633 / 79.881 ms
- Distinct applied command values: 2824
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.589 / 16.651 / 34.375 / 4165.944 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — final_v9_spin_ego1p2_1

- Wall duration: 80.810 s
- Timer callbacks / command frames / guidance frames: 3835 / 3834 / 3294
- Command FPS (ROS simulation time / active wall time): 59.891 / 56.735 Hz
- Guidance FPS (active wall time): 52.432 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.697 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.623 / 0.541 / 1.060 / 19.324 ms
- Command interval mean / P50 / P95 / max: 17.626 / 16.054 / 26.833 / 96.172 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 72.293 / 66.667 / 116.667 / 133.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.774 / 1.096 / 9.397 / 22.677 ms (steady clock)
- DDS publish call mean / P95 / max: 0.063 / 0.246 / 2.890 ms (steady clock)

#### Command bridge — final_v9_spin_ego1p2_1

- Wall duration: 80.696 s
- Received / published control frames: 3834 / 3834
- Published control FPS (ROS simulation time / active wall time): 59.891 / 56.735 Hz
- Command stamp interval mean / P50 / P95 / max: 16.697 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.074 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.626 / 16.055 / 26.712 / 96.149 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.091 / 0.087 / 0.118 / 1.674 ms (steady clock)
