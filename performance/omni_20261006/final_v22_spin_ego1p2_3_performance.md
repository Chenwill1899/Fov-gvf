
### Run final_v22_spin_ego1p2_3

- Started: 2026-10-07 02:08:11 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v22_spin_ego1p2_3

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 66.623 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 57.683 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.194 / 15.711 / 17.773 / 81.276 ms
- Distinct applied command values: 2940
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.158 / 16.114 / 32.413 / 4065.528 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — final_v22_spin_ego1p2_3

- Wall duration: 80.252 s
- Received / published control frames: 3839 / 3839
- Published control FPS (ROS simulation time / active wall time): 59.969 / 58.133 Hz
- Command stamp interval mean / P50 / P95 / max: 16.675 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.202 / 15.930 / 23.158 / 83.642 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.087 / 0.083 / 0.108 / 1.915 ms (steady clock)

#### Controller — final_v22_spin_ego1p2_3

- Wall duration: 80.359 s
- Timer callbacks / command frames / guidance frames: 3840 / 3839 / 3298
- Command FPS (ROS simulation time / active wall time): 59.969 / 58.132 Hz
- Guidance FPS (active wall time): 53.888 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.675 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.573 / 0.522 / 0.969 / 3.850 ms
- Command interval mean / P50 / P95 / max: 17.202 / 15.933 / 23.217 / 83.619 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 71.710 / 66.667 / 116.667 / 133.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.685 / 2.436 / 7.008 / 14.007 ms (steady clock)
- DDS publish call mean / P95 / max: 0.056 / 0.142 / 1.568 ms (steady clock)
