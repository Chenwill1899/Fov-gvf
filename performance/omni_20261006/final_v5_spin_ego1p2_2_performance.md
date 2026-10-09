
### Run final_v5_spin_ego1p2_2

- Started: 2026-10-06 22:39:16 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v5_spin_ego1p2_2

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 66.501 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 57.788 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.144 / 15.828 / 18.124 / 79.494 ms
- Distinct applied command values: 2713
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.897 / 16.437 / 33.300 / 4083.764 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — final_v5_spin_ego1p2_2

- Wall duration: 80.241 s
- Timer callbacks / command frames / guidance frames: 3838 / 3837 / 3296
- Command FPS (ROS simulation time / active wall time): 59.938 / 58.272 Hz
- Guidance FPS (active wall time): 53.984 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.684 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.590 / 0.529 / 1.028 / 3.245 ms
- Command interval mean / P50 / P95 / max: 17.161 / 15.731 / 26.417 / 87.632 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 69.787 / 66.667 / 116.667 / 133.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.618 / 0.791 / 9.985 / 23.829 ms (steady clock)
- DDS publish call mean / P95 / max: 0.063 / 0.409 / 2.239 ms (steady clock)

#### Command bridge — final_v5_spin_ego1p2_2

- Wall duration: 80.133 s
- Received / published control frames: 3837 / 3836
- Published control FPS (ROS simulation time / active wall time): 59.938 / 58.257 Hz
- Command stamp interval mean / P50 / P95 / max: 16.684 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.069 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.161 / 15.738 / 26.212 / 87.833 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.087 / 0.083 / 0.112 / 1.236 ms (steady clock)
