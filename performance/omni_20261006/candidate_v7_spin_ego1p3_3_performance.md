
### Run candidate_v7_spin_ego1p3_3

- Started: 2026-10-06 22:50:35 CST
- Scene: `/home/starry/isaac-data/EGO1P3/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `42`
- Manual input: `trace`

#### Isaac command application — candidate_v7_spin_ego1p3_3

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 66.988 / 64.017 s
- Applied simulation control frames: 3843
- Runtime FPS (simulation time / wall time): 60.031 / 57.369 Hz
- Isaac frame interval mean / P50 / P95 / max: 17.291 / 15.929 / 18.277 / 81.352 ms
- Distinct applied command values: 2908
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 21.583 / 16.275 / 33.608 / 4225.399 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — candidate_v7_spin_ego1p3_3

- Wall duration: 81.143 s
- Timer callbacks / command frames / guidance frames: 3707 / 3706 / 3164
- Command FPS (ROS simulation time / active wall time): 57.891 / 55.797 Hz
- Guidance FPS (active wall time): 50.765 Hz
- Command interval in ROS time mean / P50 / P95 / max: 17.274 / 16.667 / 16.667 / 350.000 ms
- Avoidance compute mean / P50 / P95 / max: 3.176 / 2.430 / 7.016 / 387.005 ms
- Command interval mean / P50 / P95 / max: 17.922 / 15.869 / 25.813 / 388.051 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 42.878 / 33.333 / 83.333 / 116.667 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 5.015 / 3.802 / 11.182 / 387.937 ms (steady clock)
- DDS publish call mean / P95 / max: 0.064 / 0.401 / 1.476 ms (steady clock)

#### Command bridge — candidate_v7_spin_ego1p3_3

- Wall duration: 81.034 s
- Received / published control frames: 3706 / 3703
- Published control FPS (ROS simulation time / active wall time): 57.891 / 55.752 Hz
- Command stamp interval mean / P50 / P95 / max: 17.274 / 16.667 / 16.667 / 350.000 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.913 / 0.000 / 0.000 / 350.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.922 / 15.874 / 25.807 / 388.141 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.087 / 0.084 / 0.110 / 1.630 ms (steady clock)
