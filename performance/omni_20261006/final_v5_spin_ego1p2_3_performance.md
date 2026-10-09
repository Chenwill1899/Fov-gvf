
### Run final_v5_spin_ego1p2_3

- Started: 2026-10-06 22:40:37 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — final_v5_spin_ego1p2_3

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 66.134 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 58.109 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.069 / 15.790 / 17.799 / 79.574 ms
- Distinct applied command values: 2719
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 22.720 / 16.423 / 32.867 / 3932.293 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — final_v5_spin_ego1p2_3

- Wall duration: 79.709 s
- Timer callbacks / command frames / guidance frames: 3840 / 3840 / 3298
- Command FPS (ROS simulation time / active wall time): 59.969 / 58.060 Hz
- Guidance FPS (active wall time): 54.280 Hz
- Command interval in ROS time mean / P50 / P95 / max: 16.675 / 16.667 / 16.667 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.712 / 0.533 / 1.500 / 8.729 ms
- Command interval mean / P50 / P95 / max: 17.224 / 15.815 / 25.504 / 581.115 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 69.679 / 66.667 / 116.667 / 133.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 2.750 / 0.868 / 9.295 / 23.455 ms (steady clock)
- DDS publish call mean / P95 / max: 0.057 / 0.203 / 1.547 ms (steady clock)

#### Command bridge — final_v5_spin_ego1p2_3

- Wall duration: 79.602 s
- Received / published control frames: 3840 / 3840
- Published control FPS (ROS simulation time / active wall time): 59.969 / 58.059 Hz
- Command stamp interval mean / P50 / P95 / max: 16.675 / 16.667 / 16.667 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.065 / 0.000 / 0.000 / 16.667 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.224 / 15.794 / 25.412 / 581.268 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.086 / 0.083 / 0.110 / 0.835 ms (steady clock)
