
### Run final_v5_spin_ego1p2_1

- Started: 2026-10-06 22:35:11 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v5_spin_ego1p2_1

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 66.788 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 57.540 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.240 / 16.030 / 18.040 / 79.797 ms
- Distinct applied command values: 2892
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.593 / 16.462 / 32.906 / 4248.085 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — final_v5_spin_ego1p2_1

- Wall duration: 80.512 s
- Timer callbacks / command frames / guidance frames: 3838 / 3837 / 3296
- Command FPS (ROS simulation time / active wall time): 59.938 / 57.946 Hz
- Guidance FPS (active wall time): 53.613 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.684 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.723 / 0.535 / 1.759 / 23.852 ms
- Command interval mean / P50 / P95 / max: 17.257 / 16.223 / 22.820 / 84.536 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 70.247 / 66.667 / 116.667 / 133.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.817 / 2.513 / 7.306 / 24.486 ms (steady clock)
- DDS publish call mean / P95 / max: 0.059 / 0.227 / 1.094 ms (steady clock)

#### Command bridge — final_v5_spin_ego1p2_1

- Wall duration: 80.407 s
- Received / published control frames: 3837 / 3837
- Published control FPS (ROS simulation time / active wall time): 59.938 / 57.946 Hz
- Command stamp interval mean / P50 / P95 / max: 16.684 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.048 / 0.000 / 0.000 / 33.333 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.257 / 16.178 / 22.763 / 84.688 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.090 / 0.086 / 0.113 / 1.467 ms (steady clock)
