
### Run final_v5_spin_ego1p3_3

- Started: 2026-10-06 22:41:58 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v5_spin_ego1p3_3

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 65.388 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 58.772 Hz
- Isaac frame interval mean / P50 / P95 / max: 16.877 / 15.559 / 17.658 / 79.027 ms
- Distinct applied command values: 2841
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 20.616 / 15.908 / 31.900 / 4103.267 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — final_v5_spin_ego1p3_3

- Wall duration: 79.130 s
- Timer callbacks / command frames / guidance frames: 3618 / 3617 / 3079
- Command FPS (ROS simulation time / active wall time): 56.500 / 55.795 Hz
- Guidance FPS (active wall time): 51.008 Hz
- Command interval in ROS time mean / P50 / P95 / max: 17.699 / 16.667 / 16.667 / 333.333 ms
- Avoidance compute mean / P50 / P95 / max: 3.898 / 2.691 / 8.833 / 284.226 ms
- Command interval mean / P50 / P95 / max: 17.923 / 15.841 / 23.845 / 304.007 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 38.367 / 33.333 / 66.667 / 100.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 5.723 / 4.220 / 11.657 / 303.926 ms (steady clock)
- DDS publish call mean / P95 / max: 0.053 / 0.140 / 1.348 ms (steady clock)

#### Command bridge — final_v5_spin_ego1p3_3

- Wall duration: 79.025 s
- Received / published control frames: 3617 / 3607
- Published control FPS (ROS simulation time / active wall time): 56.500 / 55.641 Hz
- Command stamp interval mean / P50 / P95 / max: 17.699 / 16.667 / 16.667 / 333.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 1.544 / 0.000 / 0.000 / 333.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.923 / 15.850 / 23.901 / 303.923 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.083 / 0.080 / 0.105 / 1.781 ms (steady clock)
