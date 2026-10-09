
### Run final_v22_spin_ego1p2_2

- Started: 2026-10-07 02:06:50 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v22_spin_ego1p2_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 66.225 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 58.030 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.093 / 15.738 / 17.832 / 81.567 ms
- Distinct applied command values: 2874
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.547 / 16.193 / 32.329 / 4203.802 ms (sampled on the 60 Hz OmniGraph tick)

#### Command bridge — final_v22_spin_ego1p2_2

- Wall duration: 80.030 s
- Received / published control frames: 3839 / 3839
- Published control FPS (ROS simulation time / active wall time): 59.969 / 58.476 Hz
- Command stamp interval mean / P50 / P95 / max: 16.675 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.101 / 15.908 / 22.561 / 86.063 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.088 / 0.085 / 0.110 / 1.223 ms (steady clock)

#### Controller — final_v22_spin_ego1p2_2

- Wall duration: 80.039 s
- Timer callbacks / command frames / guidance frames: 3840 / 3839 / 3298
- Command FPS (ROS simulation time / active wall time): 59.969 / 58.476 Hz
- Guidance FPS (active wall time): 54.025 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.675 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.579 / 0.532 / 0.943 / 3.289 ms
- Command interval mean / P50 / P95 / max: 17.101 / 15.915 / 22.689 / 86.202 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 69.052 / 66.667 / 116.667 / 150.000 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.710 / 2.355 / 6.919 / 14.036 ms (steady clock)
- DDS publish call mean / P95 / max: 0.054 / 0.095 / 1.941 ms (steady clock)
