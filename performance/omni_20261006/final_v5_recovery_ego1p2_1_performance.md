
### Run final_v5_recovery_ego1p2_1

- Started: 2026-10-06 22:43:18 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v5_recovery_ego1p2_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 73.288 / 72.017 s
- Applied simulation control frames: 4323
- Runtime FPS (simulation time / wall time): 60.028 / 58.987 Hz
- Isaac frame interval mean / P50 / P95 / max: 16.831 / 15.618 / 17.684 / 79.177 ms
- Distinct applied command values: 2309
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 29.820 / 16.351 / 32.743 / 4148.077 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — final_v5_recovery_ego1p2_1

- Wall duration: 87.329 s
- Timer callbacks / command frames / guidance frames: 4276 / 4276 / 3193
- Command FPS (ROS simulation time / active wall time): 59.361 / 58.340 Hz
- Guidance FPS (active wall time): 46.977 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.846 / 16.667 / 16.667 / 583.333 ms
- Avoidance compute mean / P50 / P95 / max: 1.146 / 0.474 / 1.710 / 620.628 ms
- Command interval mean / P50 / P95 / max: 17.141 / 15.557 / 23.423 / 626.269 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 69.976 / 66.667 / 116.667 / 133.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 3.147 / 1.167 / 8.516 / 626.216 ms (steady clock)
- DDS publish call mean / P95 / max: 0.055 / 0.151 / 1.752 ms (steady clock)

#### Command bridge — final_v5_recovery_ego1p2_1

- Wall duration: 87.221 s
- Received / published control frames: 4276 / 4274
- Published control FPS (ROS simulation time / active wall time): 59.361 / 58.313 Hz
- Command stamp interval mean / P50 / P95 / max: 16.846 / 16.667 / 16.667 / 583.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.386 / 0.000 / 0.000 / 583.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.141 / 15.488 / 23.319 / 626.347 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.085 / 0.082 / 0.108 / 1.490 ms (steady clock)
