
### Run candidate_v11_spin_ego1p3_1

- Started: 2026-10-06 23:20:03 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — candidate_v11_spin_ego1p3_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 66.755 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 57.569 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.217 / 15.865 / 18.350 / 79.873 ms
- Distinct applied command values: 3071
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 20.247 / 16.155 / 31.705 / 4225.204 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — candidate_v11_spin_ego1p3_1

- Wall duration: 80.545 s
- Received / published control frames: 3834 / 3834
- Published control FPS (ROS simulation time / active wall time): 59.891 / 57.976 Hz
- Command stamp interval mean / P50 / P95 / max: 16.697 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.209 / 0.000 / 0.000 / 33.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.248 / 16.125 / 23.141 / 89.063 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.086 / 0.083 / 0.109 / 0.507 ms (steady clock)

#### Controller — candidate_v11_spin_ego1p3_1

- Wall duration: 80.653 s
- Timer callbacks / command frames / guidance frames: 3835 / 3834 / 3293
- Command FPS (ROS simulation time / active wall time): 59.891 / 57.976 Hz
- Guidance FPS (active wall time): 53.825 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.697 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 2.492 / 2.682 / 6.182 / 27.681 ms
- Command interval mean / P50 / P95 / max: 17.248 / 16.141 / 23.172 / 89.003 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 41.892 / 33.333 / 83.333 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 4.402 / 4.056 / 10.322 / 39.629 ms (steady clock)
- DDS publish call mean / P95 / max: 0.058 / 0.209 / 3.584 ms (steady clock)
