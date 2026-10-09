
### Run paper_strict_final

- Started: 2026-09-24 14:54:33 CST
- Scene: `/home/starry/isaac-data/EGO1P1/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `84`
- Manual input: `trace`

#### Isaac command application — paper_strict_final

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 6.090 / 6.017 s
- Applied simulation control frames: 363
- Runtime FPS (simulation time / wall time): 60.332 / 59.606 Hz
- Isaac frame interval mean / P50 / P95 / max: 15.248 / 13.483 / 16.231 / 75.268 ms
- Distinct applied command values: 1
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — paper_strict_final

- Wall duration: 20.374 s
- Timer callbacks / command frames / guidance frames: 302 / 301 / 200
- Command FPS (ROS simulation time / active wall time): 50.000 / 54.699 Hz
- Guidance FPS (active wall time): 56.247 Hz
- Command interval in ROS time mean / P50 / P95 / max: 20.000 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.002 / 0.002 / 0.003 / 0.006 ms
- Command interval mean / P50 / P95 / max: 18.282 / 13.803 / 29.190 / 91.102 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 25.083 / 33.333 / 33.333 / 33.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 0.064 / 0.057 / 0.081 / 1.008 ms (steady clock)
- DDS publish call mean / P95 / max: 0.043 / 0.058 / 0.978 ms (steady clock)

#### Command bridge — paper_strict_final

- Wall duration: 20.264 s
- Received / published control frames: 301 / 301
- Published control FPS (ROS simulation time / active wall time): 50.000 / 54.700 Hz
- Command stamp interval mean / P50 / P95 / max: 20.000 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 18.282 / 13.818 / 29.227 / 91.054 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.084 / 0.080 / 0.109 / 0.170 ms (steady clock)

### Run paper_strict_verified

- Started: 2026-09-24 15:01:15 CST
- Scene: `/home/starry/isaac-data/EGO1P1/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`
- ROS domain: `84`
- Manual input: `trace`

#### Isaac command application — paper_strict_verified

- Result: MANUAL_TIMEOUT
- Wall duration / simulation duration: 5.861 / 6.017 s
- Applied simulation control frames: 363
- Runtime FPS (simulation time / wall time): 60.332 / 61.937 Hz
- Isaac frame interval mean / P50 / P95 / max: 14.588 / 12.773 / 15.747 / 73.942 ms
- Distinct applied command values: 1
- ESDF-blocked simulation frames: 0
- Applied-command change interval mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (sampled on the 60 Hz OmniGraph tick)

#### Controller — paper_strict_verified

- Wall duration: 20.161 s
- Timer callbacks / command frames / guidance frames: 302 / 301 / 200
- Command FPS (ROS simulation time / active wall time): 50.000 / 57.159 Hz
- Guidance FPS (active wall time): 55.762 Hz
- Command interval in ROS time mean / P50 / P95 / max: 20.000 / 16.667 / 33.333 / 33.333 ms
- Avoidance compute mean / P50 / P95 / max: 0.002 / 0.002 / 0.003 / 0.004 ms
- Command interval mean / P50 / P95 / max: 17.495 / 13.135 / 27.306 / 88.817 ms
- Latest-depth stamp to command mean / P50 / P95 / max: 32.583 / 33.333 / 33.333 / 33.333 ms (ROS simulation time)
- Control callback to publish mean / P50 / P95 / max: 0.061 / 0.055 / 0.086 / 0.645 ms (steady clock)
- DDS publish call mean / P95 / max: 0.040 / 0.056 / 0.638 ms (steady clock)

#### Command bridge — paper_strict_verified

- Wall duration: 20.051 s
- Received / published control frames: 301 / 301
- Published control FPS (ROS simulation time / active wall time): 50.000 / 57.163 Hz
- Command stamp interval mean / P50 / P95 / max: 20.000 / 16.667 / 33.333 / 33.333 ms (ROS simulation time)
- Controller publish stamp to bridge receive mean / P50 / P95 / max: 0.000 / 0.000 / 0.000 / 0.000 ms (ROS simulation time)
- Command receive interval mean / P50 / P95 / max: 17.494 / 13.185 / 27.450 / 88.623 ms (steady clock)
- Bridge conversion and `/sim/cmd_vel` publish mean / P50 / P95 / max: 0.091 / 0.083 / 0.109 / 1.656 ms (steady clock)
